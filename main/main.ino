#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "FlashManager.h"
#include "GPSController.h"

unsigned const int messCoreLenght = 10;
float messageCore[messCoreLenght] = {};

//Global variables that might be useful here and there.
bool static init = 0; //This varible just helps init-type functions to be executed just once, every termination function must reset it back to 0
int sampleRate = 2; //Fixed sampling rate for the whole system (in Hz)
long lastTime = millis(); //Current time since boot up. Manages the dynamic delay
long cycleNumber = 0; //Cycle number counter. Works as an ID for each data package
bool continuityPyros[10] = {}; 
float powderChambTemp[4] = {}; 
float kalmanState[2] = {0.0, 0.0}; 

PyroController pyro;
GPSController gps;
FlashManager flash;
SensorManager sens;
LoRaComm lora;

STAGES currentStage;

void setup() {
  Serial.begin(115200);
  pinMode(R, OUTPUT);
  digitalWrite(R, HIGH);
  currentStage = STARTUP;
}

void loop() {
  dynamicDelay();
  pyro.checkContinuityAll(continuityPyros);
  pyro.readBayTempAll(powderChambTemp);
  switch (currentStage) {
  case STARTUP:
    initStartup();
    sens.sample();
    parseData();
    printMessage();
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

void printMessage() {

  Serial.print("Message: ");
  for(int unsigned i = 0; i <= messCoreLenght + 1; i++) {
    Serial.print(messageCore[i], 1);
    Serial.print(", ");
  } 
  for(int unsigned i = 0; i < 10; i++) {
    Serial.print(continuityPyros[i]);
    Serial.print(", ");
  }
  for(int unsigned i = 0; i < 4; i++) {
    Serial.print(powderChambTemp[i],0);
    Serial.print(", ");
  }
  for(int unsigned i = 0; i < 2; i++) {
    Serial.print(kalmanState[i],0);
    Serial.print(", ");
  }
  Serial.print(digitalRead(21));
  Serial.println("");
}

void initIdle() {
  //Set sensor characteristics for IDLEING
  if(init) {
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(HIGH_RATE);
    init = false;
  }
}

void messageAppend(float info, bool reset = false) {
  static int messCounter = 0;
  if(reset) messCounter = 0;
  messageCore[messCounter] = info;
  messCounter++;
}

void parseData() {
  for (unsigned int i = 0; i <= messCoreLenght; i++) {
    messageCore[i] = 0.0;
  }
  messageAppend(cycleNumber, true);
  messageAppend(millis()/1000.0);
  messageAppend(sens.accData[0]);
  messageAppend(sens.accData[1]);
  messageAppend(sens.accData[2]);
  messageAppend(sens.alt);
  messageAppend(sens.prss);
  messageAppend(sens.euler[0]);
  messageAppend(sens.euler[1]);
  messageAppend(sens.euler[2]);
  messageAppend(sens.maxAlt);   
  messageAppend(sens.refPressure);
}

/*  Init boards neccesary systems
    It only works the first time it is called ever so sensors are not accidently rebooted
    This code would usually go in the setup function but as it is part of a rocket-stage is better to create a function for it
*/
void initStartup() {
  static bool startupinitializer = true; //Variable that protects this function from beeing called more than once
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
    
    /*
      Setup sensor modes:
      Low barometer for the 0 height reference pressure
      NDOF because why not
    */
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(IMUONLY_NDOF);
  }
}

/*
  Delays the running of the code without actually stopping it
*/
void dynamicDelay() {
  while(int(millis() - lastTime) < (1000 / sampleRate)) {
    delay(1);
  }
  lastTime = millis();
  cycleNumber++;
}

void startupTermination() {
  /*
    Terminate conditions:
    * startup when GPS has more than 5 sat fixed
    * IMU calibration is complete
    * 500 pressure samples where taken    
  */

  if(gps.getFixes() > 5) {
    sens.setReferencePressure();
    currentStage = IDLE; //Move to the next stage (Idle, naturally)
  }
}

void idleTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is higer than 5g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  /*   if(sens.acc_raw[2] > 5.0) {
    //BOOSTING characteristics
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(MID_RATE);
    currentStage = BOOSTING;
  } */
}

void boostTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is lower than 1.2g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  /*   if(sens.acc_raw[2] < 1.2) {
    //COASTING characteristics
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(IMUONLY_IMU, isFusion = true);
    currentStage = BOOSTING;
  } */
}

void coastTermination() {
  static int counter = 0;
  /*
    Coasting actions hier
  */
  
  /*Check for stage finalization condition:
    * The distance from the apogee is more than 3 meters for 5 iterations in a row  
  */
  if((sens.maxAlt - sens.alt) > 3) counter++;
  else counter=0;
  if(counter > 5) {
    //Drogue descent characteristics
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(MID_RATE);
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
  if(sens.alt < 500) {
    //Main descent characteristics
    sens.setBaroMode(HIGH_RATE);
    sens.setIMUMode(HIGH_RATE);
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
  if(sens.deltaAlt < 10) {
    timer = millis();
    enDelay = true;
  }

  if(millis() > 5) {
    //Main descent characteristics
    sens.setBaroMode(HIGH_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_AMG);
    //Fire main ignition pyrochannels
    //pyro.firePyro(3,'a');
  }
  sens.setBaroMode(LOW_RATE);
  sens.imu.setMode(OPERATION_MODE_IMUPLUS);
}