# ARCH2025-Roo

Roo is controlled over a WiFi network hosted by an ESP32. Driving is controlled by a L298 motor driver using PWM signals from the ESP32. Servo's are used to control the gimbal, using pwm directly from the ESP32.
================================================================================================
Pin Assignments:
ESP32 - L298
G23 - IN4
G25 - IN3
G26 - IN2
G27 - IN1
G12 - ENA
G14 - ENB
================================================================================================
ESP32 - GY85
G21 - SDA
G22 - SCL
================================================================================================
ESP32 - ESP32-CAM (Resets)
G18 - EN (Transistor)[Gimbal]
G19 - EN (Transistor)[Static]
G36 -    (Transistor)[ESP32 RST]
================================================================================================
ESP32 - Servo Motors
G16 - SIG (Vertical)
G17 - SIG (Horizontal)
================================================================================================
Wiring Diagram
![RooWiringV2](https://github.com/user-attachments/assets/f8aaf0cd-b4f8-4a04-869f-6129025d584e)

================================================================================================
