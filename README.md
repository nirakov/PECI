# PECI-G13: The Heart Box

### Final project for the Computer Engineering degree – Group 13 (University of Aveiro)

#### Overview: 
The Heart Box is a modular system for real-time physiological monitoring, designed for indoor cycling sessions (spinning classes) and stationary bike training. It collects heart rate and body temperature data using PPG sensors, smartwatches, and thermal cameras. The system integrates an Android app, smartwatch app, ESP32 microcontrollers, and a cloud backend for both individual and group usage.

#### [Webiste](https://carolineribeiro19.github.io/THB)  |  [Video](https://youtu.be/PlrUbAjPF5k)

## Team
* [Caroline Ribeiro](https://github.com/CarolineRibeiro19)
* [João Rodrigues](https://github.com/joaoamrodrigues)
* [Júlia Abrantes](https://github.com/JuliaAbrantes)
* [Mateus Fonte](https://github.com/mateus-fonte)
* [Nicole Rakov](https://github.com/nirakov)
* [Theo Paschôa](https://github.com/thpaschoa)
* [Supervisor: Prof. José Maria Fernandes](https://www.ua.pt/pt/p/10319434)

## Technologies
* Microcontrollers: ESP32, ESP32-CAM
* Sensors: MLX90640 (thermal camera), PPG (DFRobot)
* Mobile App: Android (Kotlin, Jetpack Compose)
* Communication: BLE, MQTT (Mosquitto broker)
* Cloud Platform: Microsoft Azure (Docker-based backend)
* Backend Stack: Node-RED, InfluxDB, Grafana
* Other: OpenCV, DataStore, RTC, Docker

# Project Structure
* PECI/
* ├── ESP (ESP32 source code PPG & MLX90640 )
* ├── HeartRateMonitor (smartwatch android app)
* ├── PeciMobileApp (Main smartphone Android application )
* ├── website 
* └── README.md

## Features

### Android App
* User profile setup with automatic HRmax estimation
* Real-time BPM and temperature display
* Heart rate zone calculation and tracking
* Individual and Group workout modes
* MQTT publishing to cloud backend
* Local data persistence (offline mode)
* Robust auto-reconnection for BLE/WebSocket

### Smartwatch App
* Real-time heart rate acquisition using PPG
* BLE broadcasting to Android app
* Minimalistic circular UI with Jetpack Compose
* Power-efficient and responsive design

### Cloud & Visualization
* Azure VM with Dockerized services: Mosquitto, Node-RED, InfluxDB, Grafana
* MQTT topics
* Dynamic dashboards per user/group with Grafana
* Real-time visualization for instructors and teams

## Use Cases
#### Individual:
Home training with BLE-only feedback and local analysis
#### Group:
Gym/spinning classes with real-time instructor dashboards and team rankings

## Getting Started

### Requirements: 
* Android Studio
* ESP32 + PPG sensor and/or MLX90640
* USB power bank
* MQTT Broker (Mosquitto)
* Docker + Azure VM or local backend

## License
This project was developed for academic purposes and is licensed under the MIT License.
