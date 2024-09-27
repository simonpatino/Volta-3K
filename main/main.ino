#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "FlashManager.h"
#include "GPSController.h"
/* #include "arduino_freertos.h"
#include "avr/pgmspace.h" */

float message[18] = {};

PyroController pyro;
GPSController gps;
FlashManager flash;
SensorManager sensor;
LoRaComm lora;

long lastTime = 0;
float lastPress = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting all systems...");
/*   if(!lora.begin()) {
    while(1) {
      Serial.println("LORA NOT CONNECTED");
      delay(1000);
    }
  } */
  if(!sensor.begin()) {
    while(1) {
      Serial.println("IMU OR BARO NOT FOUND");
      delay(1000);
    }
  }
  if(!flash.begin()) {
    while(1) {
      Serial.println("FLASH NOT FOUND");
      delay(1000);
    }
  }
  if(!gps.begin()) {
    while(1) {
      Serial.println("GPS NOT FOUND");  
      delay(1000);
    }
  }
  pyro.begin();
  Serial.println("All systems initialized");

  stageTransition(IDLE);
}

void loop() {
  sensor.readSensors();
  delay(50); 
}

void stageTransition(STAGES stage) {
  if(stage == IDLE) {
    sensor.setBaroMode(LOW_RATE);
    sensor.setIMUMode(HIGH_RATE);
    sensor.imu.setMode(OPERATION_MODE_ACCGYRO);
  } else if (stage == BOOSTING) {
    sensor.setBaroMode(MID_RATE);
    sensor.setIMUMode(MID_RATE);
    sensor.imu.setMode(OPERATION_MODE_ACCGYRO);
  } else if (stage == COASTING) {
    sensor.setBaroMode(MID_RATE);
    sensor.imu.setMode(OPERATION_MODE_NDOF);
  } else if (stage == DROGUE_DESCENT) {
    sensor.setBaroMode(MID_RATE);
    sensor.setIMUMode(MID_RATE);
    sensor.imu.setMode(OPERATION_MODE_ACCGYRO);
  } else if (stage == MAIN_DESCENT) {
    sensor.setBaroMode(HIGH_RATE);
    sensor.setIMUMode(HIGH_RATE);
    sensor.imu.setMode(OPERATION_MODE_AMG);
  } else if (stage == TOUCH_DOWN) {
    sensor.setBaroMode(LOW_RATE);
    sensor.imu.setMode(OPERATION_MODE_IMUPLUS);
  }
}