# Apex BioMechanics — Low-Cost EMG-Controlled Prosthetic Hand

E164 project by **APEX Mechanics**: Dylan Adili, Amy Enayati, Alex Bilello.

A prosthetic hand that performs a grab when the user flexes their forearm. We build this hand with under $105 using a MyoWare 2.0 muscle sensor, a Teensy microcontroller, and an SG90 servo.

## How it works
1. Gel electrodes on the forearm pick up muscle signals; the MyoWare 2.0 amplifies, filters, and rectifies them.
2. The Teensy samples the signal at 1 kHz and smooths it with a 100 ms moving average.
3. Calibration: the user relaxes, then fully flexes, so the signal is scaled 0–100% for that person.
4. Above 20% the hand closes; below 15% it opens (the gap prevents flickering).
5. The servo pulls the finger tendons; elastic cords return the fingers when it releases.

## Hardware
- Teensy 3.2
- MyoWare 2.0 Muscle Sensor + gel electrodes
- SG90 micro servo
- 5–6 V power supply for the servo

## Wiring
| From | To |
|---|---|
| MyoWare ENV | Teensy pin 14 (A0) |
| MyoWare + | Teensy 3.3V |
| MyoWare − | Teensy GND |
| Servo orange (signal) | Teensy pin 2 |
| Servo red | Supply + (5–6 V) |
| Servo brown | Supply − |
| Supply − | Teensy GND (shared ground — required) |

## Running the code
1. Install the Arduino IDE and Teensy support (Teensyduino).
2. Open `apex_hand/apex_hand.ino`, select **Tools → Board → Teensy 3.2**, and upload.
3. Open the Serial Monitor and follow the relax/squeeze calibration.
4. Flex your muscles to close the hand, relax to open. 

## Settings you can tune (in `apex_hand.ino`)
- `OPEN_ANGLE` / `CLOSED_ANGLE` — servo angles for open and closed (depending on how you position the motors)
- `ON_THRESHOLD` / `OFF_THRESHOLD` — flex level that closes / reopens the hand
- `MAX_CLOSE_RATE` — how fast the hand moves

**Safety:** while electrodes are on someone, run the laptop on battery (unplugged from the wall).
