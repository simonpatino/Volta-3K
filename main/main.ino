/*
  TODO:
  - Write the Kalman Filter
  - Include the PID test algorithm, log its output
  - Change some stage termination conditions to be according to the state given by the filter
  - Enable a SD card reading mode
  - Write all log data into the SD Card after touchdown
  - Enable command reading on idle for flight termination
  - Include CATO detection and instant flight termination when the state looks wrong
  - Include Software on the Loop using an SD card test file for easier testing
*/

#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "FlashManager.h"
#include "GPSController.h"

const int messCoreLenght = 10;
float messageCore[messCoreLenght] = {};

//Global variables that might be useful here and there.
int sampleRate = 2;        //Fixed sampling rate for the whole system (in Hz)
long lastTime = millis();  //Current time since boot up. Manages the dynamic delay
long cycleNumber = 0;      //Cycle number counter. Works as an ID for each data package
bool continuityPyros[10] = {};
float powderChambTemp[4] = {};
float kalmanState[2] = { 0.0, 0.0 };  //Position and velocity state

PyroController pyro;
GPSController gps;
FlashManager flash;
SensorManager sens;
LoRaComm lora;

STAGES currentStage;

void setup() {
  Serial.begin(115200);
  pinMode(R, OUTPUT);
  digitalWrite(R, LOW);
  currentStage = STARTUP;
}

void loop() {
  dynamicDelay();
  pyro.checkContinuityAll(continuityPyros);
  pyro.readBayTempAll(powderChambTemp);
  switch (currentStage) {
    case STARTUP:
      startUpInit();
      sens.sample();
      parseData();
      lora.transmitData(messageCore, 0x00);
      serialPrintMessage();
      //startupTermination();
      break;
    case IDLE:
      idleInit();
      /*
        More wanted actions here
      */
      sens.sample();
      parseData();
      serialPrintMessage();
      idleTermination();
      break;

    case BOOSTING:
      boostInit();
      /*
      More Boosting actions here
      */
      sens.sample();
      parseData();
      serialPrintMessage();
      boostTermination();
      break;

    case COASTING:
      coastInit();
      /*
      More wanted actions here
    */
      sens.sample();
      parseData();
      serialPrintMessage();
      coastTermination();
      break;

    case DROGUEDESCENT:
      drogueInit();
      /*
      More wanted actions here
    */
      sens.sample();
      parseData();
      serialPrintMessage();
      drogueTermination();
      break;

    case MAINDESCENT:
      mainDescentInit();
      /*
      More wanted actions here
    */
      sens.sample();
      parseData();
      serialPrintMessage();
      //mainDescentTermination();
      break;

    case TOUCHDOWN:
      sens.sample();
      parseData();
      serialPrintMessage();
      touchDownInit();
      break;

    // Add more cases as needed
    default:
      // Code to execute if none of the above cases match
      break;
  }
}

void serialPrintMessage() {
  Serial.print("Message: ");
  for (int i = 0; i < messCoreLenght; i++) {
    Serial.print(messageCore[i], 1);
    Serial.print(", ");
  }
  for (int i = 0; i < 10; i++) {
    Serial.print(continuityPyros[i]);
    Serial.print(", ");
  }
  for (int i = 0; i < 4; i++) {
    Serial.print(powderChambTemp[i], 0);
    Serial.print(", ");
  }
  for (int i = 0; i < 2; i++) {
    Serial.print(kalmanState[i], 0);
    Serial.print(", ");
  }
  Serial.println("");
}

void messageAppend(float info, bool reset = false) {
  /* 
    Enables the parsing algorithm to be easier to change the other way easier 
  */
  static int messCounter = 0;
  if (reset) messCounter = 0;
  if (messCounter < messCoreLenght)
    messageCore[messCounter] = info;
  messCounter++;
}

void parseData() {
  /*
    Organizes the data in a float array for easier printing or radio-communicating it
  */
  for (int i = 0; i < messCoreLenght; i++) {
    messageCore[i] = 0.0;
  }
  messageAppend(cycleNumber, true);
  messageAppend(millis() / 1000.0);
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

void dynamicDelay() {
  /*
    Delays the running of the code without actually stopping it
  */
  while (int(millis() - lastTime) < (1000 / sampleRate)) {
    delay(1);
  }
  lastTime = millis();
  cycleNumber++;
}

/*
  * From here on, every init function initializes an specific stage of the flight and its only called once.
  * Every termination function terminates a function, meaning, it detects that the stage termination condiction was matched to 
  asign the current stage as the next one in order.
*/

void startUpInit() {
  /*  
    Init boards neccesary systems
    It only works the first time it is called ever so sensors are not accidently rebooted
    This code would usually go in the setup function but as it is part of a rocket-stage is better to create a function for it
  */
  static bool startupInitializer = true;  //Variable that protects this function from beeing called more than once
  if (startupInitializer) {
    Serial.println("Starting all systems...");
    if (!gps.begin()) {
      while (1) {
        Serial.println("GPS NOT FOUND");
        delay(1000);
      }
    }
    if (!lora.begin()) {
      while (1) {
        Serial.println("LORA NOT CONNECTED");
        delay(1000);
      }
    }
    if (!sens.begin()) {
      while (1) {
        Serial.println("IMU OR BARO NOT FOUND");
        delay(1000);
      }
    }
    if (!flash.begin()) {
      while (1) {
        Serial.println("FLASH NOT FOUND");
        delay(1000);
      }
    }
    pyro.begin();
    Serial.println("All systems initialized");
    startupInitializer = false;

    /*
      Setup sensor modes:
      Low barometer for the 0 height reference pressure
      NDOF because why not
    */
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(IMUONLY_NDOF);
  }
}

void startupTermination() {
  /*
    Terminate conditions:
    * startup when GPS has more than 5 sat fixed
    * IMU calibration is complete
    * 500 pressure samples where taken    
  */

  if (gps.getFixes() > 5) {
    sens.setReferencePressure();
    currentStage = IDLE;  //Move to the next stage (Idle, naturally)
  }
}

void idleInit() {
  //Set sensor characteristics for IDLEING
  static bool idleInitializer = true;
  if (idleInitializer) {
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(HIGH_RATE);
    idleInitializer = false;
  }
}

void idleTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is higer than 5g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  if (sens.linAccData[2] > 5.0) {
    currentStage = BOOSTING;
  }
}

void boostInit() {
  static bool boostInitializer = true;
  if (boostInitializer) {
    /* Set boosting characteristics */
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(MID_RATE);
    boostInitializer = false;
  }
}

void boostTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is lower than 1.2g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  if (sens.linAccData[2] < 0) {
    currentStage = COASTING;
  }
}

void coastInit() {
  static bool coastInitializer = true;
  if (coastInitializer) {
    /* Set coasting characteristics */
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(IMUONLY_IMU);
    coastInitializer = false;
  }
}

void coastTermination() {
  /*
    Check for stage finalization condition:
    * The distance from the apogee is more than 3 meters for 5 iterations in a row  
  */
  static int counter = 0;
  if ((sens.maxAlt - sens.alt) > 3) counter++;
  else counter = 0;
  if (counter > 5) {
    currentStage = DROGUEDESCENT;
  }
}

void drogueInit() {
  static bool drogueInitializer = true;
  if (drogueInitializer) {
    /* Set drogue descent characteristics*/
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(IMUONLY_IMU);
    pyro.firePyro(1, 'a');
    pyro.firePyro(1, 'b');
    pyro.firePyro(2, 'a');
    pyro.firePyro(2, 'b');
    drogueInitializer = false;
  }
}

void drogueTermination() {
  /*Check for stage finalization condition:
    * The altitude is lower than 500mts
  */
  if (sens.alt < 1200) {
    currentStage = DROGUEDESCENT;
  }
}

void mainDescentInit() {
  static bool mainDescentInitializer = true;
  if (mainDescentInitializer) {
    /* Main descent characteristics */
    sens.setBaroMode(HIGH_RATE);
    sens.setIMUMode(HIGH_RATE);
    pyro.firePyro(4, 'a');
    pyro.firePyro(4, 'b');
    pyro.firePyro(5, 'a');
    pyro.firePyro(5, 'b');
    mainDescentInitializer = false;
  }
}

void mainDescentTermination() {
  static int counterDT = 0;
  /*
    Wanted actions hier
  */

  /*Check for stage finalization condition:
    * Delta altitude is lower than 2 mts for 5 iterations in row
    * TODO: change condition for z-axis velocity is lower than 1 m/s for 25 iterations in a row
  */
  if (sens.deltaAlt < 2) counterDT++;
  else counterDT = 0;
  if (counterDT > 5) {
    currentStage = TOUCHDOWN;
  }
}

void touchDownInit() {
  static bool touchDownInit = true;
  if (touchDownInit) {
    /* Touch down descent characteristics */
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_AMG);
    touchDownInit = false;
  }
}