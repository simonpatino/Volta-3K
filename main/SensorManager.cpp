#include <Arduino.h>
#include "SensorManager.h"
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>

SensorManager::SensorManager() {

}

bool SensorManager::begin() {
    if (!baro.begin(0x77, &Wire2)) {
        return 0;
    }
    if (!imu.begin()) {
        return 0;
    }
    Serial.println("Sensors initialized!");
    return 1;
}

void SensorManager::readSensors(float bmeVariables[], float bnoVariables[]) {
    temp = baro.readTemperature();       // °C
    prss = baro.readPressure() / 100.0F; // hPa
    alt = baro.readAltitude(SEALEVELPRESSURE_HPA); // m
    humty = baro.readHumidity();          // %

    imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
    euler_angles[0] = euler.x(); // ° (Euler angles)
    euler_angles[1] = euler.y(); // °
    euler_angles[2] = euler.z(); // °
}
