# Volta-3K
This GitHub project is designed to provide a comprehensive flight control system for a rocket aiming to reach an altitude of 3 km. The code includes modules for calculating the rocket's apogee, monitoring various flight parameters, and managing the deployment of the payload at the optimal time.

## Hardware

| Component               | Description                                                       |
|-------------------------|-------------------------------------------------------------------|
| Teensy 4.1 (DPU)        | The main processing unit for managing all flight control tasks.   |
| BNO085                  | An inertial measurement unit (IMU) for precise orientation data.  |
| Adafruit Flash Card XTSD| High-speed storage for logging flight data.                       |
| BME 280                 | Measures atmospheric pressure, humidity, and temperature.         |
| 10 Pyrotechnical Channels | Controls pyrotechnic devices for deployment and separation events. |
| LoRa RFM95W             | Long-range communication module for telemetry and control.        |

## Features

- **Apogee Calculation**: Determines the highest point of the rocket's flight.
- **Flight Variables Monitoring**: Continuously tracks and logs critical flight data.
- **Payload Deployment**: Automates the payload release process based on real-time flight analysis.

Ideal for aerospace enthusiasts and professionals seeking a robust solution for mid-altitude rocket missions.
