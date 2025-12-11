# ESP32-S3 USB Gamepad Servo Controller

I used ChatGPT5.1 to assist in stting up the github repo. This is readme is auto generated.  I hope you find this project useful as an end to end demo of joystick to servo.
ISSUES: You need to power the ESP32S3 first then plug in the gamepad.  The gamepad I have boots in a Vendor Specific mode, and I have to hold H button for 5 seconds for it to switch into a generic HID controller. It's a model T43 controller sold as "RivalPlay" on Amazon at the time of writing.

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

