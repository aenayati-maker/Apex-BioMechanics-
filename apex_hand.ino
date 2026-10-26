// =====================================================================
//  APEX Mechanics — E164 Low-Cost EMG-Controlled Prosthetic Hand
//  Dylan Adili, Amy Enayati, Alex Bilello
//
//  Hardware: Teensy 3.6 + MyoWare 2.0 Muscle Sensor + 1 SG90 servo
//  Function: one grip pattern (grab). Flex -> hand closes, relax -> opens.
//
//  Signal chain (see project plan, Engineering analysis/methods):
//    gel electrodes -> MyoWare 2.0 (amplify, filter, rectify, envelope)
//    -> Teensy A0 (sampled at 1 kHz, 12-bit) -> 100 ms moving average
//    -> normalize to the user's calibrated rest/max -> threshold
//    (with hysteresis) -> servos pull tendons to close the fingers
//
//  WIRING
//    MyoWare ENV  -> Teensy A0
//    MyoWare +    -> Teensy 3.3V   (NOT 5V: Teensy 3.6 analog pins are
//    MyoWare -    -> Teensy GND     not 5V tolerant)
//    Servo signal -> pin 2 (one servo pulls all five finger tendons)
//    Servo power  -> separate 5-6V supply (NOT the Teensy). Tie that
//                    supply's GND to Teensy GND.
//    Recalibrate button (optional) -> pin 12 to GND
//
//  SAFETY: while electrodes are on someone, run the laptop on battery
//  (unplugged from the wall), per MyoWare's guidance.
//
//  SERIAL (115200 baud) — open Serial Plotter to see the signal live
//    'c' = recalibrate (do this at the start and if muscles fatigue)
//    'r' = toggle printing raw ADC counts instead of plotter data
// =====================================================================

#include <Servo.h>

// ---------------------------- Pins -----------------------------------
const int EMG_PIN        = A0;
const int CAL_BUTTON_PIN = 12;   // INPUT_PULLUP, press = LOW
const int LED_PIN        = 13;   // on-board LED, lit while grip is active

const int NUM_FINGERS = 1;   // number of servos (all tendons on one pulley)
const int SERVO_PINS[NUM_FINGERS] = {2};

// Servo angles (degrees) for open and closed. Tune each finger on the
// bench during the one-finger prototype test — tendon length and pulley
// size change these. If a finger closes the wrong way, swap its values.
int OPEN_ANGLE[NUM_FINGERS]   = {150};
int CLOSED_ANGLE[NUM_FINGERS] = {10};

// --------------------------- Sampling --------------------------------
const uint32_t SAMPLE_PERIOD_US = 1000;  // 1 kHz hardware timer
const int      WINDOW           = 100;   // 100 samples = 100 ms average
const uint32_t CONTROL_PERIOD_MS = 10;   // control loop runs at 100 Hz
const uint32_t PRINT_PERIOD_MS   = 20;   // serial output at 50 Hz

// -------------------------- Calibration ------------------------------
const uint32_t REST_MS   = 3000;  // relax for 3 s
const uint32_t FLEX_MS   = 3000;  // squeeze for 3 s
const uint32_t SETTLE_MS = 700;   // ignore start of each phase (reaction time)
const float    MIN_RANGE = 60.0;  // flex must beat rest by this many ADC counts

// --------------------------- Detection -------------------------------
// Normalized signal: 0 = resting, 1 = calibrated max flex.
// Hysteresis: close above ON, only reopen below OFF, so the hand
// doesn't chatter when the signal hovers near one threshold.
const float    ON_THRESHOLD  = 0.20;
const float    OFF_THRESHOLD = 0.15;
const uint32_t DEBOUNCE_MS   = 30;   // state must hold this long to switch

// ---------------------------- Motion ---------------------------------
// Position is a fraction: 0 = fully open, 1 = fully closed.
// Rate limit keeps fingers from snapping (pinch-point safety) while
// staying fast: 3.0/s means a full close in ~0.33 s.
const float MAX_CLOSE_RATE = 2.0;

// Stretch goal: if true, grip closes in proportion to how hard the user
// flexes instead of all-or-nothing.
const bool PROPORTIONAL_MODE = false;

// ======================================================================
//  Sampling (runs in a timer interrupt, independent of loop())
// ======================================================================
IntervalTimer sampleTimer;

volatile uint16_t ring[WINDOW];      // last 100 raw samples
volatile uint32_t ringSum   = 0;     // running sum of ring[]
volatile int      ringIdx   = 0;
volatile uint16_t latestRaw = 0;

void sampleISR() {
  uint16_t v = analogRead(EMG_PIN);
  ringSum -= ring[ringIdx];          // drop oldest sample
  ringSum += v;                      // add newest sample
  ring[ringIdx] = v;
  ringIdx = (ringIdx + 1) % WINDOW;
  latestRaw = v;
}

// 100 ms moving average, in ADC counts (0-4095)
float readSmoothed() {
  noInterrupts();
  uint32_t s = ringSum;
  interrupts();
  return s / (float)WINDOW;
}

uint16_t readRaw() {
  noInterrupts();
  uint16_t r = latestRaw;
  interrupts();
  return r;
}

// ======================================================================
//  State
// ======================================================================
Servo fingers[NUM_FINGERS];

float restLevel = 0;      // calibrated resting level (counts)
float flexLevel = 4095;   // calibrated max flex level (counts)
bool  calibrated = false;

bool     gripActive     = false;
bool     pendingSwitch  = false;
uint32_t pendingSince   = 0;

float handPos = 0;        // current commanded position, 0 open .. 1 closed
float normSig = 0;        // latest normalized signal

bool printRaw = false;

// ======================================================================
//  Actuation
// ======================================================================
void writeHand(float pos) {
  pos = constrain(pos, 0.0, 1.0);
  for (int i = 0; i < NUM_FINGERS; i++) {
    int angle = OPEN_ANGLE[i] + pos * (CLOSED_ANGLE[i] - OPEN_ANGLE[i]);
    fingers[i].write(angle);
  }
}

// Move toward target without exceeding MAX_CLOSE_RATE
void stepHandToward(float target, float dtSec) {
  float maxStep = MAX_CLOSE_RATE * dtSec;
  float err = target - handPos;
  if (err >  maxStep) err =  maxStep;
  if (err < -maxStep) err = -maxStep;
  handPos += err;
  writeHand(handPos);
}

// ======================================================================
//  Calibration
//  Rest baseline = mean of the smoothed signal while relaxed.
//  Max = peak of the smoothed signal while flexing hard.
//  Redone per session because amplitude depends on the user and on
//  electrode placement (plan ref [29]).
// ======================================================================
float measure(uint32_t durationMs, bool takePeak) {
  uint32_t start = millis();
  while (millis() - start < SETTLE_MS) { delay(1); }

  float sum = 0, peak = 0;
  uint32_t n = 0;
  start = millis();
  while (millis() - start < durationMs - SETTLE_MS) {
    float s = readSmoothed();
    sum += s;
    if (s > peak) peak = s;
    n++;
    delay(CONTROL_PERIOD_MS);
  }
  return takePeak ? peak : (n ? sum / n : 0);
}

void countdown(const char *msg) {
  Serial.println(msg);
  for (int i = 3; i > 0; i--) {
    Serial.print("  ");
    Serial.println(i);
    digitalWrite(LED_PIN, HIGH); delay(150);
    digitalWrite(LED_PIN, LOW);  delay(850);
  }
}

void calibrate() {
  // Hand stays open and detection is off during calibration
  calibrated = false;
  gripActive = false;
  pendingSwitch = false;
  handPos = 0;
  writeHand(0);

  while (true) {
    countdown("CAL: get ready to RELAX your forearm...");
    Serial.println("CAL: relax now");
    restLevel = measure(REST_MS, false);

    countdown("CAL: get ready to SQUEEZE as hard as you can...");
    Serial.println("CAL: squeeze now");
    digitalWrite(LED_PIN, HIGH);
    flexLevel = measure(FLEX_MS, true);
    digitalWrite(LED_PIN, LOW);

    Serial.print("CAL: rest = ");  Serial.print(restLevel, 1);
    Serial.print("  max = ");      Serial.println(flexLevel, 1);

    if (flexLevel - restLevel >= MIN_RANGE) break;

    // Too little difference: electrodes probably misplaced or loose
    Serial.println("CAL: flex barely differs from rest. Check electrode "
                   "placement/contact and try again.");
    delay(1500);
  }

  calibrated = true;
  Serial.println("CAL: done. Flex to grab, relax to open.");
}

// ======================================================================
//  Detection: normalize, then threshold with hysteresis + debounce
// ======================================================================
float normalize(float smoothed) {
  float n = (smoothed - restLevel) / (flexLevel - restLevel);
  return constrain(n, 0.0, 1.0);
}

void updateGripState(float n, uint32_t now) {
  // Which state does the signal currently ask for?
  bool want = gripActive ? (n > OFF_THRESHOLD) : (n > ON_THRESHOLD);

  if (want == gripActive) {
    pendingSwitch = false;
    return;
  }
  if (!pendingSwitch) {
    pendingSwitch = true;
    pendingSince = now;
  } else if (now - pendingSince >= DEBOUNCE_MS) {
    gripActive = want;
    pendingSwitch = false;
    digitalWrite(LED_PIN, gripActive ? HIGH : LOW);
  }
}

// ======================================================================
//  Setup / loop
// ======================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

  analogReadResolution(12);
  analogReadAveraging(4);   // light hardware averaging per sample

  for (int i = 0; i < WINDOW; i++) ring[i] = 0;
  for (int i = 0; i < NUM_FINGERS; i++) fingers[i].attach(SERVO_PINS[i]);
  writeHand(0);             // start open

  sampleTimer.begin(sampleISR, SAMPLE_PERIOD_US);
  delay(1500);              // let the average window fill, open Serial

  calibrate();
}

void handleInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') calibrate();
    if (c == 'r' || c == 'R') printRaw = !printRaw;
  }
  if (digitalRead(CAL_BUTTON_PIN) == LOW) {
    delay(30);                                  // debounce the button
    if (digitalRead(CAL_BUTTON_PIN) == LOW) {
      while (digitalRead(CAL_BUTTON_PIN) == LOW) {}
      calibrate();
    }
  }
}

void printStatus() {
  if (printRaw) {
    Serial.println(readRaw());
    return;
  }
  // Serial Plotter format, all in percent so they share one scale
  Serial.print("signal:");  Serial.print(normSig * 100, 1);
  Serial.print(",on:");     Serial.print(ON_THRESHOLD * 100, 0);
  Serial.print(",off:");    Serial.print(OFF_THRESHOLD * 100, 0);
  Serial.print(",grip:");   Serial.print(gripActive ? 100 : 0);
  Serial.print(",hand:");   Serial.println(handPos * 100, 1);
}

void loop() {
  static uint32_t lastControl = 0;
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  handleInput();

  if (calibrated && now - lastControl >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControl) / 1000.0;
    lastControl = now;

    normSig = normalize(readSmoothed());
    updateGripState(normSig, now);

    float target;
    if (PROPORTIONAL_MODE) {
      // Map ON_THRESHOLD..1 to open..closed while active
      target = gripActive
        ? constrain((normSig - OFF_THRESHOLD) / (1.0 - OFF_THRESHOLD), 0.0, 1.0)
        : 0.0;
    } else {
      target = gripActive ? 1.0 : 0.0;
    }
    stepHandToward(target, dt);
  }

  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    printStatus();
  }
}
