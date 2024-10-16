#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "FlashManager.h"
#include "GPSController.h"
/* #include "arduino_freertos.h"
#include "avr/pgmspace.h" */

float message[18] = {};

float maxAlt;

PyroController pyro;
GPSController gps;
FlashManager flash;
SensorManager sensor;
LoRaComm lora;

STAGES currentStage;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting all systems...");
/*   if(!gps.begin()) {
    while(1) {
      Serial.println("GPS NOT FOUND");  
      delay(1000);
    }
  }
  if(!lora.begin()) {
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
  pyro.begin();
  Serial.println("All systems initialized");
  currentStage = IDLE;

  //Set sensor characteristics for IDLEING
  sensor.setBaroMode(LOW_RATE);
  sensor.setIMUMode(HIGH_RATE);
  sensor.imu.setMode(OPERATION_MODE_ACCGYRO);
}

void IDLEStage() {
  /*
    Wanted actions hier
  */

  /*Check for stage finalization condition:
    * Acceleration along the z axis is higer than 5g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  if(sensor.acc_raw[2] > 5.0) {
    //BOOSTING characteristics
    sensor.setBaroMode(MID_RATE);
    sensor.setIMUMode(MID_RATE);
    sensor.imu.setMode(OPERATION_MODE_ACCGYRO);
    currentStage = BOOSTING;
  }
}

void BOOSTStage() {
  /*
    Wanted actions hier
  */

  /*Check for stage finalization condition:
    * Acceleration along the z axis is lower than 1.2g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  if(sensor.acc_raw[2] < 1.2) {
    //COASTING characteristics
    sensor.setBaroMode(MID_RATE);
    sensor.imu.setMode(OPERATION_MODE_NDOF);
    currentStage = BOOSTING;
  }
}

void COASTStage() {
  static int counter = 0;
  /*
    Wanted actions hier
  */
  
  /*Check for stage finalization condition:
    * The distance from the apogee is more than 3 meters for 5 iterations in a row  
  */
  if((maxAlt - sensor.alt) > 3) counter++;
  else counter=0;
  if(counter > 5) {
    //Drogue descent characteristics
    sensor.setBaroMode(MID_RATE);
    sensor.setIMUMode(MID_RATE);
    sensor.imu.setMode(OPERATION_MODE_ACCGYRO);
    //Fire main ignition pyrochannels
    //pyro.firePyro(1,'a');
  }
}

void DRODEStage() {
    /*
    Wanted actions hier
  */


  /*Check for stage finalization condition:
    * The altitude is lower than 500mts
  */
  if(sensor.alt < 500) {
    //Main descent characteristics
    sensor.setBaroMode(HIGH_RATE);
    sensor.setIMUMode(HIGH_RATE);
    sensor.imu.setMode(OPERATION_MODE_AMG);
    //Fire main ignition pyrochannels
    //pyro.firePyro(3,'a');
  }
}
void MAINDEStage() {
  static long timer = 0;
  static bool enDelay = false;
  /*
    Wanted actions hier
  */

  /*Check for stage finalization condition:
    * Altitude is lower than 10 mts and 10 seconds have passed since then
    * TODO: change condition for z-axis velocity is lower than 1 m/s for 25 iterations in a row
  */
  if(deltaAlt < 10) {
    timer = millis();
    enDelay = true;
  }
  if(millis() > 5) {
    //Main descent characteristics
    sensor.setBaroMode(HIGH_RATE);
    sensor.setIMUMode(HIGH_RATE);
    sensor.imu.setMode(OPERATION_MODE_AMG);
    //Fire main ignition pyrochannels
    //pyro.firePyro(3,'a');
  }
  sensor.setBaroMode(LOW_RATE);
  sensor.imu.setMode(OPERATION_MODE_IMUPLUS);

}
void TDStage() {
  /*
    Wanted actions hier
  */
}

void loop() {
  switch (currentStage) {
  case IDLE:
    break;
  
  case BOOSTING:
    break;
  
  case COASTING:
    break;
  
  case DROGUEDESCENT:
    break;

  case MAINDESCENT:
    break;
  
  case TOUCHDOWN:
    break;
  
  // Add more cases as needed

  default:
    // Code to execute if none of the above cases match
    break;
  }


  sensor.readSensors();
  Serial.println(sensor.acc_raw[2]);
  delay(100);
  //gps.checkGPS();
}
