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

#include "Sigma.h"
#include "KalmanFilter.h"
#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "MemoryManager.h"
#include "GPSController.h"
#include <SD.h>
#include <map>
#include <string>
#include <ArduinoEigen.h>

const int messCoreLenght = 11;
float messageCore[messCoreLenght] = {};

std::map<String, float> currentData;

//Global variables that might be useful here and there.
int sampleDelay = 1000;      //Fixed sampling delay for the whole system (in ms)
long lastTime = millis();  //Current time since boot up. Manages the dynamic delay
long cycleNumber = 0;      //Cycle number counter. Works as an ID for each data package
bool continuityPyros[10] = { false, false, false, false, false, false, false, false, false, false };
float powderChambTemp[4] = {};

//KalmanFilter variables
const int x_dim = 3;
const int z_dim = 2;
const int u_dim = 0;
const float process_variance = 0.3872;
const float pos_measurement_variance = 50.;
const float acc_measurement_variance = 10.;
VectorXf kalmanState = VectorXf::Zero(x_dim);    // All elements are 0
float alpha = 0.01, beta = 2.0, kappa = 0.;  //Sigma points generation parameters

//Rocket properties:
const double gravity = -9.8;  // m/s²
const double Cd = 0.1;        // Drag coefficient
const double rho = 1.225;     // Air density (kg/m³)
const double A = 0.01;        // Cross-sectional area (m²)
const double mass = 5.0;      // Mass (kg)


//Simulation variables
const bool IS_SIMULATION = true;  //This variable allows for software on a loop integration
const bool VERBOSE = true;        //This variable allows for explicit printing of stage change and other flight events for debugging
int simDataColumnNumber = 0;
float baroStdDev = 5;
float accelStdDev = 2;


PyroController pyro;
GPSController gps;
MemoryManager flash;
MemoryManager sd;
SensorManager sens;
LoRaComm lora;
MerweScaledSigmaPoints sigmaPoints(x_dim, alpha, beta, kappa);
KalmanFilter kf(x_dim, z_dim, u_dim, sigmaPoints);  // Creating the object with default dimensions


STAGES currentStage;

void setup() {
  Serial.begin(9600);
  pinMode(RLED, OUTPUT);
  currentStage = STARTUP;
}

void loop() {
  dynamicDelay();
  pyro.readBayTempAll(powderChambTemp);
  switch (currentStage) {
    case STARTUP:
      startUpInit();
      gps.updateGPS();
      checkCommand();
      //lora.transmitData(messageCore, messCoreLenght, 0x00);
      startupTermination();
      break;
    case IDLE:
      idleInit();
      /*
        More wanted actions here
      */
      sample();
      KFStep();
      parseData();
      //serialPrintMessage();
      //sd.logData(messageCore, messCoreLenght, kf.dataFileName);
      //idleTermination();
      break;

    case BOOSTING:
      boostInit();
      /*
      More Boosting actions here
      */
      sample();
      //parseData();
      //serialPrintMessage();
      boostTermination();
      break;

    case COASTING:
      coastInit();
      /*
      More wanted actions here
      */
      sample();
      //parseData();
      //serialPrintMessage();
      coastTermination();
      break;

    case DROGUEDESCENT:
      drogueInit();
      /*
      More wanted actions here
    */
      sample();
      //parseData();
      //serialPrintMessage();
      drogueTermination();
      break;

    case MAINDESCENT:
      mainDescentInit();
      /*
      More wanted actions here
      */
      sample();
      //parseData();
      //serialPrintMessage();
      mainDescentTermination();
      break;

    case TOUCHDOWN:
      sample();
      //parseData();
      //serialPrintMessage();
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
    Serial.print(messageCore[i], 2);
    Serial.print(", ");
  }
  /*   for (int i = 0; i < 10; i++) {
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
  } */
  Serial.println("");
}

void sample() {
  static float prevSampleTime = 0.0f;
  if (IS_SIMULATION) {
    currentData["intTime"] = 0;
    while(currentData["intTime"] < (sampleDelay/1000.)) {
      if (sd.readSimulatedData(currentData)) {
        //Les agregamos ruido gausiano de media 0 a las dos mediciones que podemos tomar
        currentData["alt"] = addGaussianNoise(currentData["realAlt"], 0, baroStdDev);
        currentData["accData2"] = addGaussianNoise(currentData["realAccData2"], 0, accelStdDev);
        currentData["intTime"] = currentData["time"] - prevSampleTime;  //Interval between to consecutive readings

        float realData[x_dim+1];
        realData[0] = currentData["time"];
        realData[1] = currentData["realAlt"];
        realData[2] = currentData["rawVel"];
        realData[3] = currentData["realAccData2"];
        sd.logData(realData, x_dim+1, sd.kfRealFileName);
      } else {
        if (VERBOSE) {
          Serial.println("Error pulling info from simulation");
          break;
        }
      }
    }
  } else {
    sens.readSensors(currentData);
    currentData["intTime"] = currentData["time"] - prevSampleTime;
    float realData[x_dim+1];
    realData[0] = currentData["time"];
    realData[1] = currentData["alt"];
    realData[2] = 0.0;
    realData[3] = currentData["linAccData2"];
    sd.logData(realData, x_dim, sd.kfRealFileName);
  }
  prevSampleTime = currentData["time"];
}

/* Used for adding gaussian noise into the "measured" variables in the SIMULATION mode
   Is intended to evaluate the Kalman Filter Performance 
*/
float addGaussianNoise(float value, float mean, float stdDev) {
  // Generar dos números aleatorios uniformes en el rango (0, 1)
  float u1 = random(1, 10000) / 10000.0;  // Convertir a rango (0, 1)
  float u2 = random(1, 10000) / 10000.0;

  // Usar la transformación Box-Muller para generar números con distribución normal
  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);

  // Escalar el resultado al nivel deseado de varianza y media
  return value + z0 * stdDev + mean;
}

/*
    What has to happen every iteration so we are able to run the Kalman Filter
    1. Given currentData["intTime"] modify the F and Q matrices
    2. Locate the sensed data into z, the measurement vector
    3. Execute KF filter prediction
    4. Execute KF filter update
    5. Save data into kf_real (if it is a simulation), kf_measu, kf_output and kf_config file
  */
void KFStep() {
  //1
  static MatrixXf Q(x_dim, x_dim);
  Q = kf.set_Q_discrete_white_noise(x_dim, currentData["intTime"], process_variance);
  //kf.set_Q(Q); //not necessary when using set_Q_discrete_white_noise;
  //2
  static VectorXf z(z_dim);
  z(0) = currentData["alt"];
  z(1) = currentData["accData2"];
  //3
  kf.predict(currentData["intTime"]);
  //4
  kf.update(z);
  //5
  kalmanState = kf.get_x();
  float configData[] = {(float)x_dim, (float)z_dim };
  float measuData[] = { z(0), 0.0, z(1)};  //As we can only measure altitude and the velocity is a hidden variable
  float outputData[] = { currentData["time"], kalmanState(0), kalmanState(1), kalmanState(2)};
  sd.logData(configData, 2, sd.kfConfigFileName);  //The 2 here is really hard coded NEEDS change
  sd.logData(outputData, x_dim+1, sd.kfOutputFileName); //+1 counting for time
  sd.logData(measuData, x_dim, sd.kfMeasuFileName);
}

// Define the state transition function
VectorXf fx(const VectorXf& x, float dt) {
  VectorXf x_new(3);  // New state vector [position, velocity, acceleration]
  float velocity = x(1);

  float dragForce = 0.5 * Cd * rho * A * velocity * velocity * (velocity > 0 ? 1 : -1);

  x_new(0) = x(0) + x(1) * dt + 0.5 * x(2) * dt * dt;

  x_new(1) = x(1) + x(2) * dt;

  x_new(2) = x(2);


  x_new(2) = gravity - (dragForce / mass);
  x_new(1) = velocity + (x_new(2) * dt);
  x_new(0) = x(0) + (velocity * dt);

  return x_new;
}
// Define the measurement function
VectorXf hx(const VectorXf& x) {
  MatrixXf H(z_dim, x_dim);
  H << 1, 0, 0,
    0, 0, 1;
  return H * x;
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
  if (IS_SIMULATION) {
    messageAppend(currentData["iter"], true);
    messageAppend(currentData["time"]);
  } else {
    messageAppend(cycleNumber, true);
    messageAppend(millis() / 1000.0);
  }
  messageAppend(currentData["accData0"]);
  messageAppend(currentData["accData1"]);
  messageAppend(currentData["accData2"]);
  messageAppend(currentData["alt"]);
  messageAppend(currentData["prss"]);
  messageAppend(currentData["euler0"]);
  messageAppend(currentData["euler1"]);
  messageAppend(currentData["euler2"]);
  messageAppend(currentData["maxAlt"]);
}

void dynamicDelay() {
  /*
    Delays the running of the code without actually stopping it
  */
  while (int(millis() - lastTime) < sampleDelay && !(IS_SIMULATION)) {
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
  digitalWrite(RLED, HIGH);
  if (startupInitializer) {
    if (VERBOSE) {
      Serial.println("Starting all systems...");
    }
    if (!gps.begin()) {
      while (1) {
        if (VERBOSE) {
          Serial.println("GPS NOT FOUND");
        }
        delay(1000);
      }
    }
    if (!lora.begin()) {
      while (1) {
        if (VERBOSE) {
          Serial.println("LORA NOT CONNECTED");
        }
        delay(1000);
      }
    }
    if (!sens.begin()) {
      while (1) {
        if (VERBOSE) {
          Serial.println("IMU OR BARO NOT FOUND");
        }
        delay(1000);
      }
    }
    if (!flash.begin('f', CS_FLASH)) {
      while (1) {
        if (VERBOSE) {
          Serial.println("FLASH NOT FOUND");
        }
        delay(1000);
      }
    }
    if (!sd.begin('s', CS_SD)) {
      while (1) {
        if (VERBOSE) {
          Serial.println("COULDN'T REACH SD CARD");
        }
        delay(1000);
      }
    }
    if (IS_SIMULATION) {
      randomSeed(2);
      if (VERBOSE) {
        Serial.println("This is a simulation");
      }
      simDataColumnNumber = sd.startSimulationData();
      if (simDataColumnNumber == 0) {
        if (VERBOSE) {
          Serial.println("Simulation file not found");
        }
      } else if (simDataColumnNumber == -1) {
        if (VERBOSE) {
          Serial.println("Simulation file is empty");
        }
      } else if (simDataColumnNumber >= 1) {
        if (VERBOSE) {
          Serial.print("Simulation file initialized, # columns: ");
          Serial.println(simDataColumnNumber);
        }
      }
    } else {
      //sens.setReferencePressure();
    }
    pyro.begin();
    if (VERBOSE) {
      Serial.println("All systems initialized");
    }

    /*
      *- Initialize Kalman Filter for state tracking.
    */
    VectorXf initial_state(x_dim);
    MatrixXf initial_covariance(x_dim, x_dim);
    initial_state << 0, 0, 0;
    initial_covariance << 500, 0, 0,
      0, 500, 0,
      0, 0, 500;
    kf.initialize_state(initial_state, initial_covariance);
    MatrixXf H(z_dim, x_dim);
    MatrixXf R(z_dim, z_dim);
    H << 1, 0, 0,
      0, 0, 1;
    R << pos_measurement_variance, 0,
      0, acc_measurement_variance;
    kf.set_fx(fx);
    kf.set_hx(hx);
    kf.set_H(H);
    kf.set_R(R);
    /*
      Setup sensor modes:
      Low barometer for the 0 height reference pressure
      NDOF because why not
    */
    sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(IMUONLY_NDOF);

    //So we dont re-initialize it
    startupInitializer = false;
  }
}

void startupTermination() {
  /*
    Terminate conditions:
    * startup when GPS has more than 5 sat fixed
    * IMU calibration is complete
    * 500 pressure samples where taken    
    * is a simulation
  */

  if (gps.getFixes() >= 4 || IS_SIMULATION) {
    if (VERBOSE) {
      Serial.println("Start up terminated");
    }
    currentStage = IDLE;  //Move to the next stage (Idle, naturally)
  }
}

void idleInit() {
  //Set sensor characteristics for IDLEING
  static bool idleInitializer = true;
  if (idleInitializer) {
    if (VERBOSE) {
      Serial.println("Idle mode initialized");
    }
    /*     sens.setBaroMode(LOW_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_AMG); */
    idleInitializer = false;
  }
}

void idleTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is higer than 5g
    TODO: DO NOT USE z acceleration but net acceleration
  */
  if (currentData["accData2"] > 5.0) {
    if (VERBOSE) {
      Serial.println("Idle terminated");
    }
    currentStage = BOOSTING;
  }
}

void boostInit() {
  static bool boostInitializer = true;
  if (boostInitializer) {
    /* Set boosting characteristics */
    if (VERBOSE) {
      Serial.println("Boosting initialized");
    }
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
  if (currentData["accData2"] < 0) {
    if (VERBOSE) {
      Serial.println("Boosting terminated");
    }
    currentStage = COASTING;
  }
}

void coastInit() {
  static bool coastInitializer = true;
  if (coastInitializer) {
    /* Set coasting characteristics */
    if (VERBOSE) {
      Serial.println("Coasting initialized");
    }
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
  if ((currentData["maxAlt"] - currentData["alt"]) > 3) counter++;
  else counter = 0;
  if (counter > 5) {
    if (VERBOSE) {
      Serial.println("Coasting terminated");
    }
    currentStage = DROGUEDESCENT;
  }
}

void drogueInit() {
  static bool drogueInitializer = true;
  if (drogueInitializer) {
    /* Set drogue descent characteristics*/
    if (VERBOSE) {
      Serial.println("Drogue descent initialized");
    }
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
  if (currentData["alt"] < 1200) {
    if (VERBOSE) {
      Serial.println("Drogue descent terminated");
    }
    currentStage = MAINDESCENT;
  }
}

void mainDescentInit() {
  static bool mainDescentInitializer = true;
  if (mainDescentInitializer) {
    /* Main descent characteristics */
    if (VERBOSE) {
      Serial.println("Main descent initialized");
    }
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
    * If altitude is lower than 10 mts
    * TODO: change condition for z-axis velocity is lower than 1 m/s for 5 iterations in a row using KF
  */
  if (currentData["alt"] < 10) counterDT++;
  else counterDT = 0;
  if (counterDT >= 5) {
    if (VERBOSE) {
      Serial.println("Main descent terminated");
    }
    currentStage = TOUCHDOWN;
  }
}

void touchDownInit() {
  static bool touchDownInit = true;
  if (touchDownInit) {
    /* Touch down descent characteristics */
    if (VERBOSE) {
      Serial.println("Touch down");
    }
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_AMG);
    touchDownInit = false;
  }
}


/*
  Esta funcion tiene uso especialmente cuando se está en IDLE en el launchpad para enviar comandos a la computadora a bordo
  Esta funcion recive ese comando y lleva a cabo trasmisiones respectivas a cada comando
*/
void checkCommand() {
  lora.checkReceive();
  byte command = lora.lastCommand;
  if (command == 0x03) {
    //Comando de enviar datos de GPS
    if (VERBOSE) {
      Serial.println("I'll send GPS DATA");
    }
    lora.lastCommand = 0x00;
    gps.updateGPS();
    lora.transmitGPS(gps.getFixes(), gps.getLatitude(), gps.getLongitude());
  } else if (command == 0x04) {
    //Comando de enviar datos de canales pirotécnicos
    if (VERBOSE) {
      Serial.println("I'll send Pyro data");
    }
    pyro.checkContinuityAll(continuityPyros);
    lora.transmitPyroInfo(continuityPyros);
    lora.lastCommand = 0x00;
  } else if (command == 0x05) {
    if (VERBOSE) {
      Serial.println("I'll send chamber data");
    }
    lora.lastCommand = 0x00;
  }
}