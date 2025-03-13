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
    float refPressure = 753.46;
    bool saveCalibration();
    void restoreCalibration();
    void checkCalibrationStatus();
    void putToSleep();
  private:
    uint8_t offsets[22];  // Buffer to store calibration data
    bool calibrationSaved = false;  // Flag to check if we have valid offsets
    const int EEPROM_FLAG_ADDR = 0;      // Address to store flag (indicates valid calibration data)
    const int EEPROM_OFFSET_ADDR = 1;    // Start address for offset data (22 bytes)
};

#endif
