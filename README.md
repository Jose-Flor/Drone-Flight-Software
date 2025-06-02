# 🚁 Drone-Flight-Software

An advanced Python-based flight control system for a **quadcopter** drone, featuring autonomous waypoint missions, real-time obstacle detection using OpenCV, and return-to-launch (RTL) safety.

![Screenshot](https://github.com/user-attachments/assets/e95a005d-7436-408c-9ace-ba0e39b719bf)

---

## 📌 Features

- 🧭 Autonomous waypoint navigation with DroneKit
- 🚫 Obstacle detection using a live OpenCV camera feed
- ✈️ GPS-guided flight control with takeoff, navigation, and RTL
- 🔋 Battery fail-safe for auto-return
- 🖥️ Real-time telemetry and flight feedback

---

## ⚙️ Technologies Used

- Python 3.x
- DroneKit-Python
- OpenCV
- NumPy, Imutils
- MAVLink (via USB or telemetry)
- ArduCopter firmware (quadcopter configuration)

---

## 🛠️ Hardware Setup

- 4 brushless motors + ESCs
- Flight controller (e.g., Pixhawk or Cube running ArduPilot)
- Companion computer (Jetson Nano, Raspberry Pi, or PC)
- USB/CSI camera
- GPS and telemetry modules
- 3S/4S LiPo battery and power module

---

## 🚀 Quick Start

### 1. Clone and Install Dependencies

```bash
git clone https://github.com/your-username/Drone-Flight-Software.git
cd Drone-Flight-Software
pip install dronekit opencv-python imutils numpy

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/Drone-Flight-Software.git
cd Drone-Flight-Software

