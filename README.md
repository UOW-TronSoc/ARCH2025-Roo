# ARCH2025-Roo

Roo is controlled over a WiFi network hosted by an ESP32. Driving is controlled by a L298 motor driver using PWM signals from the ESP32. Servo's are used to control the gimbal, using pwm directly from the ESP32.
---
### Pin Assignments

#### ESP32 - L298
| ESP32 Pin | L298 Pin  |
|-----------|----------|
| G23       | IN4      |
| G25       | IN3      |
| G26       | IN2      |
| G27       | IN1      |
| G12       | ENA      |
| G14       | ENB      |

---

#### ESP32 - GY85
| ESP32 Pin | GY85 Pin |
|-----------|---------|
| G21       | SDA     |
| G22       | SCL     |

---

#### ESP32 - ESP32-CAM (Resets)
| ESP32 Pin | ESP32-CAM Function     |
|-----------|------------------------|
| G18       | EN (Transistor) [Gimbal] |
| G19       | EN (Transistor) [Static] |
| G36       | (Transistor) [ESP32 RST] |

---

#### ESP32 - Servo Motors
| ESP32 Pin | Servo Motor Function  |
|-----------|----------------------|
| G16       | SIG (Vertical)       |
| G17       | SIG (Horizontal)     |

---
Wiring Diagram
![RooWiringV2](https://github.com/user-attachments/assets/f8aaf0cd-b4f8-4a04-869f-6129025d584e)

