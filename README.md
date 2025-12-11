# ESP32-S3 USB Gamepad Servo Controller

I used ChatGPT5.1 to assist in stting up the github repo. This is readme is mostly auto generated.  
I hope you find this project useful as an end to end demo of joystick to servo.

ISSUES: You need to power the ESP32S3 first then plug in the gamepad.  The gamepad I have boots in a Vendor Specific mode, and I have to hold H button for 5 seconds for it to switch into a generic HID controller. It's a model T43 controller sold as "RivalPlay" on Amazon at the time of writing.

You need to have ESP-IDF installed in VS-Code, and know how to build projects.  You'll need to adapt the code to suit your specific hardware, as I'm using a custom board.

_____________________________________________________________________________________________________
AI generated (mostly):


This project demonstrates how to use an **ESP32-S3** as a **USB Host for HID gamepad input**, driving:

✔ 4 servos via LEDC PWM  
✔ 2 relays with gamepad buttons  
✔ Digital sensors as inputs

Specifically tested with **ESP-IDF v5.5.1** and a wired USB gamepad in HID mode.

---

## 🚀 Features

- USB Host support on the ESP32-S3
- HID Host driver to receive controller input
- Four-axis mapping:
  - Left stick → BASE / SHOULDER
  - Right stick → ELBOW / JAW
- Buttons mapped to relays:
  - **A/B** → MOTOR1 start/stop
  - **X/Y** → MOTOR2 start/stop
- Clean and tested HID report decoding
- LEDC PWM 14-bit servos (50 Hz)
- Easy to extend to other HID controllers

---

## 📦 Repository Structure

esp32s3-usb-gamepad-servos/
├── main/
│ ├── main.c
│ ├── CMakeLists.txt
├── sdkconfig.defaults
├── CMakeLists.txt
└── idf_component.yml


---

## 🛠 Requirements

- ESP-IDF v5.5.1
- ESP32-S3 module (e.g. ESP32-S3-WROOM-1 N4R2)
- S3 must be wired to a USB connector directly via the dedicated D=/D- pins, and provide 5V power to the controller.
- USB gamepad wired in **HID mode**


---

## 📥 Building Instructions
pen a terminal in VS-Code, ESP-IDF extension:
idf.py set-target esp32s3
idf.py fullclean
idf.py build
idf.py flash monitor

🔌 Wiring
✔ USB to ESP32-S3

Use the USB-OTG pins (GPIO19, GPIO20) via a USB-A host receptacle.

✔ Servos
Servo	GPIO
BASE	GPIO4
ELBOW	GPIO5
SHOULDER	GPIO6
JAW	GPIO7
✔ Relays
Relay	GPIO
MOTOR1	GPIO17
MOTOR2	GPIO18
✔ Sensors
Sensor	GPIO
SENSOR1	GPIO41
SENSOR2	GPIO42

🎮 HID Report Format (tested controller)
Report length: 8 bytes

Byte 0: Main buttons
  bit0 = X | bit1 = A | bit2 = B | bit3 = Y
  bit4 = L1 | bit5 = R1 | bit6 = L2 | bit7 = R2

Byte 1: System buttons
  bit0 = Minus (-)
  bit1 = Plus (+)
  bit4 = Home (H)
  T & O have no effect in HID reports

Byte 2: D-pad (HAT)
  0x88 = neutral
  0x00 = up
  0x44 = down
  0x66 = left
  0x22 = right

Byte 3 = LX (0 = left, 255 = right)
Byte 4 = LY (0 = up,   255 = down)
Byte 5 = RX (0 = left, 255 = right)
Byte 6 = RY (0 = up,   255 = down)
Byte 7 = unused/reserved

📌 Notes / Tips

Many USB controllers have multiple modes; some expose non-HID vendor modes by default.
If you see no HID reports, consult controller documentation or try holding mode buttons
to force Qualcomm/Fusion/HID mode.

This code is written for HID-compliant devices (interface class = 0x03).

⭐ Future Improvements

Non planned - This is intended to help you get something going
