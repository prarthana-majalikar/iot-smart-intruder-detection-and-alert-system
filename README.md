# 🛡️ Smart Intruder Detection and Alert System (IoT)

## 📌 Project Overview
The **Smart Intruder Detection and Alert System** is an Internet of Things (IoT)–based security solution designed to detect unauthorized intrusions in real time and issue immediate alerts. The system integrates sensors, embedded hardware, and communication protocols to monitor motion, identify intrusion events, and notify users promptly.

This project was developed as part of **CS 244P – Internet of Things** at the **University of California, Irvine**.

---

## 🎯 Objectives
- Detect motion-based intrusions using IoT sensors
- Process sensor data efficiently on embedded hardware
- Trigger alerts immediately upon detecting suspicious activity
- Demonstrate an end-to-end IoT pipeline (device → communication → alerting)
- Build a modular and extensible system architecture

---

## 🧠 System Architecture
The system follows a layered IoT architecture:

1. **Sensing Layer**  
   - Motion detection using sensors such as PIR

2. **Processing Layer**  
   - Embedded microcontroller (e.g., ESP32) processes sensor input
   - Local logic determines intrusion events and reduces false positives

3. **Communication Layer**  
   - Wireless communication via Wi-Fi
   - HTTP or MQTT-based data transmission

4. **Alert Layer**  
   - Triggers alerts (e.g., buzzer, notification, or server-side signal)

---

## 🔧 Technologies Used
### Hardware
- ESP32
- Gyroscope
- Buzzer / LED indicators

### Software
- C++ 
- Arduino / PlatformIO

### Protocols & Tools
- Wi-Fi
- HTTP / MQTT
- Git & GitHub
- VS Code
- PlatformIO

---

## ✅ Features
- Real-time intrusion detection
- Immediate alert generation
- Low-latency response
- Modular and extensible codebase
- Designed for scalability and future enhancements

---

## 📂 Project Structure
```text
├── src/                 # Embedded source code
├── include/             # Header files
├── lib/                 # External libraries
├── test/                # Test cases 
├── docs/                # Documentation and diagrams
├── platformio.ini       # PlatformIO configuration
├── README.md            # Project documentation

```
## ▶️ How to Run the Project

- Clone the repository
- Open the project in VS Code with PlatformIO installed.
- Connect the hardware components according to the wiring setup.
- Build and upload the firmware
- Monitor serial output


## 🚀 Future Enhancements

- Camera-based intrusion verification
- ML-based anomaly detection
- Multi-zone security support
