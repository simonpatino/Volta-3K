#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "FlashManager.h"
#include "GPSController.h"

float message[18] = {};

//Global variables that might be useful here and there.
float maxAlt;
bool static init = 0; //This varible just helps init-type functions to be executed just once, every termination function must reset it back to 0
int sampleRate = 5; //Fixed sampling rate for the whole system (in Hz)
long lastTime = millis(); //Current time since boot up. Manages the dynamic delay

PyroController pyro;
GPSController gps;
FlashManager flash;
SensorManager sens;
LoRaComm lora;

STAGES currentStage;

void setup() {
  Serial.begin(115200);
  currentStage = STARTUP;
}

void loop() {
  dynamicDelay();
  switch (currentStage) {
  case STARTUP:
    initStartup();
    sens.sample();
    parseData();
    startupTermination();
    break;
  case IDLE:
    initIdle();
    /*
      Wanted actions hier
    */

    idleTermination();
    break;
  
  case BOOSTING:
    /*
      Boosting actions hier
    */

    boostTermination();
    break;
  
  case COASTING:
    /*
      Wanted actions hier
    */

    coastTermination();
    break;
  
  case DROGUEDESCENT:
    /*
      Wanted actions hier
    */

    drogueTermination();
    break;

  case MAINDESCENT:

    descentTermination();
    break;
  
  case TOUCHDOWN:
    break;
  
  // Add more cases as needed

  default:
    // Code to execute if none of the above cases match
    break;
  }
}

void initIdle() {
  //Set sensor characteristics for IDLEING
  if(init) {
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_ACCGYRO);
    init = false;
  }
}

void initIdle() {
  //Set sensor characteristics for IDLEING
  if(init) {
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_ACCGYRO);
    init = false;
  }
}

void parseData() {

}


/*  Init boards neccesary systems
    It only works the first time it is called ever so sensors are not accidently rebooted
    This code would usually go in the setup function but as it is part of a rocket-stage is better to create a function for it
*/
void initStartup() {
  static startupinitializer = true //Variable that protects this function from beeing called more than once
  if(startupinitializer) {
    Serial.println("Starting all systems...");
    if(!gps.begin()) {
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
    }
    if(!sens.begin()) {
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
    startupinitializer = false;
  }
}

/*
  Delays the running of the code without actually stopping it
*/
void dynamicDelay() {
  while(millis() - lastTime < (1000 / sampleRate)) {
    delay(1)
  }
  lastTime = millis();
}

void startupTermination() {
  /*
    Terminate conditions:
    * startup when GPS has more than 5 sat fixed
    * IMU calibration is complete
    * 500 pressure samples where taken    
  */
  if(gps.getFixes()) {}
}

void idleTermination() {
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

void boostTermination() {
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

void coastTermination() {
  static int counter = 0;
  /*
    Coasting actions hier
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

void drogueTermination() {
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
void descentTermination() {
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