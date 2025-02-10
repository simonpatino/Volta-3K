#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include "Constants.h"
#include <map>

class SensorManager {
  public:
    SensorManager();
    bool begin();
    Adafruit_BNO055 imu = Adafruit_BNO055(-1, 0x28, &Wire2);
    Adafruit_BME280 baro;  // I2C
    void readSensors(std::map<String, float> &dataDict);
    void setReferencePressure();
    void setBaroMode(ODR_MODES mode);
    bool setIMUMode(ODR_MODES mode);
    float refPressure = 1013.25;
  private:
};

#endif
