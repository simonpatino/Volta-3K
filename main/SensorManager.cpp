#include <Arduino.h>
#include "SensorManager.h"
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include "Constants.h"
#include "Wire.h"
#include <map>

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

void SensorManager::readSensors(std::map<String, float> &dataDict) {
  static float prevAtl;
  dataDict["temp"] = baro.readTemperature();       // °C
  dataDict["prss"] = baro.readPressure() / 100.0F; // hPa
  dataDict["alt"] = baro.readAltitude(refPressure); // hPa
  dataDict["deltaAlt"] = dataDict["alt"] - prevAtl;
  dataDict["humty"] =  baro.readHumidity(); // %;
  prevAtl = dataDict["alt"];
  if(dataDict["alt"] > dataDict["maxAlt"]) dataDict["maxAlt"] = dataDict["alt"];
  imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  dataDict["euler0"] = euler[0];
  dataDict["euler1"] =  euler[1];
  dataDict["euler2"] =  euler[2];
  imu::Vector<3> accData = imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  dataDict["accData0"] = accData[0];
  dataDict["accData1"] = accData[1];
  dataDict["accData2"] = accData[2];
  imu::Vector<3> angVelData = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  dataDict["angVelData0"] = angVelData[0];
  dataDict["angVelData1"] = angVelData[1];
  dataDict["angVelData2"] = angVelData[2];
  imu::Vector<3> linAccData = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  dataDict["linAccData0"] = linAccData[0];
  dataDict["linAccData1"] = linAccData[1];
  dataDict["linAccData2"] = linAccData[2];
  
  dataDict["time"] = millis();

  //imu::Vector<3> magData = mu.getVector(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //imu::Vector<3> gravityData = imu.getVector(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
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
  Serial.println("Reference pressure taken");
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
  /*
    The IMU can be set to multiple different standard modes according to the datasheet
    In general there are to types: Fusion and no-fusion
    Fusion modes use a KF filter to estimate multiple more variables but are limited to sense a maximum of 4g of acceleration per axis
    Non fusion modes just pull raw data and should only consider be used in the Idling so it is more realiable to detect high accelerations (boosting)
    This function allows to set the BNO055 in some of those standard modes or into other custom modes.
  */
  if(mode == IMUONLY_IMU) {
    imu.setMode(OPERATION_MODE_IMUPLUS);
    return 1;
  } else if(mode == IMUONLY_NDOF) {
    imu.setMode(OPERATION_MODE_NDOF);
    return 1;
  } 

  //Change to Config Mode:
  imu.setMode(OPERATION_MODE_CONFIG); //This mode let's us set the configuration as we want it. Very customed
  int result = 0;

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x07); // Im pretty sure this sets the I2C to the second page of the registers
  Wire2.write(0x01);
  Wire2.endTransmission();


  //This gives higher control on the BNO055 performance.
  if(mode == LOW_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_15_63HZ << 2) | ACC_RNG_2G;
  } else if(mode == MID_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_62_5HZ << 2) | ACC_RNG_8G;
  } else if(mode == HIGH_RATE) {
    result = (ACC_NORMAL << 5) | (ACC_BW_500HZ << 2) | ACC_RNG_16G;
  } else return 0;

  //Serial.println(result);

  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x08);
  Wire2.write(result);
  Wire2.endTransmission();

  //Return to page 0:
  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x07);
  Wire2.write(0x00);
  Wire2.endTransmission();

  imu.setMode(OPERATION_MODE_ACCGYRO); //Only activate acc and gyro, if mag is needed change this. This is a no fusion mode

  return 1;
}
