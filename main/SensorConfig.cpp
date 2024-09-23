#include "SensorConfig.h"
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;  // I2C
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

void setupSensors() {
    if (!bme.begin(0x77, &Wire2)) {
        Serial.println("Could not find BME280 sensor!");
        while (1);
    }
    if (!bno.begin()) {
        Serial.println("Could not find BNO055 sensor!");
        while (1);
    }
    Serial.println("Sensors initialized!");
}

void readSensors(float bmeVariables[], float bnoVariables[]) {
    bmeVariables[0] = bme.readTemperature();       // °C
    bmeVariables[1] = bme.readPressure() / 100.0F; // hPa
    bmeVariables[2] = bme.readAltitude(SEALEVELPRESSURE_HPA); // m
    bmeVariables[3] = bme.readHumidity();          // %

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    bnoVariables[0] = euler.x(); // ° (Euler angles)
    bnoVariables[1] = euler.y(); // °
    bnoVariables[2] = euler.z(); // °
}
