#include <Arduino.h>
#include "SensorManager.h"
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include "Constants.h"
#include "Wire.h"

SensorManager::SensorManager() {

}

bool SensorManager::begin() {
  Wire2.begin();
  if (!baro.begin(0x77, &Wire2)) {
    Serial.println("barof");
    return 0;
  }
  if (!imu.begin()) {
    Serial.println("imuf");
    return 0;
  }
  return 1;
}

void SensorManager::sample() {
  static float prevAtl;
  temp = baro.readTemperature();       // °C
  prss = baro.readPressure() / 100.0F; // hPa
  alt = baro.readAltitude(SEALEVELPRESSURE_HPA); // m
  deltaAlt = alt - prevAtl;
  humty = baro.readHumidity();          // %
  prevAtl = alt;

  imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  euler_angles[0] = euler.x(); // ° (Euler angles)
  euler_angles[1] = euler.y(); // °
  euler_angles[2] = euler.z(); // °
  sensors_event_t accelerometerData;
  imu.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  acc_raw[2] = accelerometerData.acceleration.z/9.8;
}

void SensorManager::setBaroMode(ODR_MODES mode) {
  if(mode == LOW_RATE) {
    //LOW DATA RATE AND POWER NEEDED | 1HZ | 220 ms refesh according to experiments
    baro.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_NONE, Adafruit_BME280::FILTER_X2, Adafruit_BME280::STANDBY_MS_250);
  } else if(mode == MID_RATE) {
    //MID NAV MODE: SEE DATASHEET PAGE 20 + HUMIDITY DISABLED | 25 Hz | 30 ms refesh according to experiments
    baro.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_NONE, Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
  } else if(mode == HIGH_RATE) {
    //HIGH GAMING MODE: SEE DATASHEET PAGE 21 | 80 Hz | 15 ms refesh according to experiments
    baro.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X4, Adafruit_BME280::SAMPLING_NONE, Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
  }
}

void SensorManager::setIMUMode(ODR_MODES mode) {
  //Change to Config Mode:
  imu.setMode(OPERATION_MODE_CONFIG);

  int result = 0;

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x07);
  Wire2.write(0x01);
  Wire2.endTransmission();

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x08);
  if(mode == LOW_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_15_63HZ << 2) | ACC_RNG_2G;
  } else if(mode == MID_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_62_5HZ << 2) | ACC_RNG_8G;
  } else if(mode == HIGH_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_500HZ << 2) | ACC_RNG_16G;
  }
  Wire2.write(result);
  Wire2.endTransmission();

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x07);
  Wire2.write(0x00);
  Wire2.endTransmission();
}
