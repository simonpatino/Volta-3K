#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include "Constants.h"

class SensorManager {
  public:
    SensorManager();
    bool begin();
    Adafruit_BNO055 imu = Adafruit_BNO055(-1, 0x28, &Wire2);
    Adafruit_BME280 baro;  // I2C
    void readSensors();
    void setBaroMode(ODR_MODES mode);
    void setIMUMode(ODR_MODES mode);
    const float SEALEVELPRESSURE_HPA = 1013.25;
    float temp, prss, alt, deltaAlt, humty;
    float euler_angles[3];
    float acc_raw[3];
    float gyro_raw[3];
  private:
};

#endif
