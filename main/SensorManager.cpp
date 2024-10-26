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
  setBaroMode(MID_RATE);
  if (!imu.begin()) {
    Serial.println("imuf");
    return 0;
  }
  return 1;
  setIMUMode(MID_RATE);
}

void SensorManager::sample() {
  static float prevAtl;
  temp = baro.readTemperature();       // °C
  prss = baro.readPressure() / 100.0F; // hPa
  alt = baro.readAltitude(refPressure); // hPa
  deltaAlt = alt - prevAtl;
  humty = baro.readHumidity();          // %
  prevAtl = alt;
  if(alt > maxAlt) maxAlt = alt;
  euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  accData = imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  angVelData = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  linAccData = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //magData = mu.getVector(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //gravityData = imu.getVector(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  /*
    The lib should be aware of the fact it is working in fusion mode or not to calculate angles an everything else by itself
  */
}

void SensorManager::setReferencePressure() {
  refPressure = 0;
  for(int i = 0; i < 50; i++) {
    //Update the pressure with the mean equation
    refPressure = (i * refPressure + (baro.readPressure() / 100.0F)) /(i+1);
  }
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

bool SensorManager::setIMUMode(ODR_MODES mode) {
  if(mode == IMUONLY_IMU) {
    imu.setMode(OPERATION_MODE_IMUPLUS);
    return 1;
  } else if(mode == IMUONLY_NDOF) {
    imu.setMode(OPERATION_MODE_NDOF);
    return 1;
  } 
  //Change to Config Mode:
  imu.setMode(OPERATION_MODE_CONFIG); //This mode let's us set the configuration we want
  int result = 0;

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x07);
  Wire2.write(0x01);
  Wire2.endTransmission();

  if(mode == LOW_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_15_63HZ << 2) | ACC_RNG_2G;
  } else if(mode == MID_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_62_5HZ << 2) | ACC_RNG_8G;
  } else if(mode == HIGH_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_500HZ << 2) | ACC_RNG_16G;
  } else return 0;

  Serial.println(result);

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x08);
  Wire2.write(result);
  Wire2.endTransmission();

  //Return to page 0:
  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x07);
  Wire2.write(0x00);
  Wire2.endTransmission();

  imu.setMode(OPERATION_MODE_ACCGYRO); //Only activate acc and gyro, if mag is needed change this

  return 1;
}
