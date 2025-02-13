/*
  TODO:
  - Include the PID test algorithm, log its output
  - Change some stage termination conditions to be according to the state given by the filter
  - Write all log data into the SD Card after touchdown (better flash-sd cowork)
  - Enable command reading on idle for flight termination
  - Include CATO detection and instant flight termination when the state looks wrong
  - Take into account than the sensed acceleration is the proper and not the net acceleration
  - Fix GPS NOT FOUND condition, maybe add a few retries
  - Transmission of stage change
  - Figure out if desable and re-enabling fusion mode requires re-calibration
*/

#include "Sigma.h" //For creating and managing the sigmapoints used in the UKF
#include "KalmanFilter.h" //For managing the KF atributes and equations
#include "SensorManager.h" //For managing the BNO055 and the BME280
#include "PyroController.h" // Manages the pyrochannels check, fire and kill capabilites
#include "LoRaComm.h" // Manages the LoRa module
#include "MemoryManager.h" // Manages both the Flash module and the Teensy 4.1 built-in microSD card slot
#include "GPSController.h" // Manages the GPS M9N
#include <map> // The last sensed data is saved in a map so it is easier  to access the data by all code sectors that might need it
#include <ArduinoEigen.h> // To interact with the Kalman Filter using matrices and vectors
#include <string>

const int messCoreLenght = 11; //Lenght of the main data packet that we'll save in memory and send through LoRa
float messageCore[messCoreLenght] = {}; //The actual data in a float array. It has an specific order

std::map<String, float> currentData; // The last recorded data
/*
  KEYS
  - iter: the number of the iteration of sensed data, from 1 and on
  - time: current time
  - temp: sensed temperature by BME280
  - prss: sensed pressure by BME280
  - realAlt: it is the "actual" altidude of the rocket, specifically used for simulation mode
  - alt: the sensed (then noisy) altitude of the rocket given by the BME280
  - deltaAlt: difference between the last sensed alt and the current alt. Should not be used and maybe should be deleted.
  - humty: humidity
  - angVelData0: X-axis gryscope reading
  - angVelData1: Y-axis gryscope reading
  - angVelData2: Z-axis gryscope reading
  - accData0: X-axis acceleration reading
  - accData1: Y-axis acceleration reading
  - realAccData2: Z-axis raw pull (with no noise) if running in simulation mode
  - accData2: Z-axis acceleration reading of the BNO055 (with noise)
    The next 6 variables can only be read if the BNO055 is set to be in Fusion Mode (using the built-in Kalman Filter)
  - euler0: X-axis rotation
  - euler1: Y-axis rotation
  - euler2: Z-axis rotation
  - linAccData0: intertial-X-axis acceleration
  - linAccData1: intertial-Y-axis acceleration
  - linAccData2: intertial-Z-axis acceleration

  For simulation only:
  - rawVel: velocity reading from the simulation, used to verify KF performance

*/

//Global variables that might be useful here and there.
int sampleDelay = 1000;      //Fixed sampling delay for the whole system (in ms)
long lastTime = millis();  //Current time since boot up. Manages the dynamic delay
long cycleNumber = 0;      //Cycle number counter. Works as an ID for each data package
bool continuityPyros[10] = { false, false, false, false, false, false, false, false, false, false }; //Array with the continuity data of the pyrochannels
float powderChambTemp[4] = {}; // Array with temperature data from the LM35

//KalmanFilter variables
const int x_dim = 3;
const int z_dim = 2;
const int u_dim = 0;
const float process_variance = 0.3872; //Q variance for set_Q_discrete_white_noise()
const float pos_measurement_variance = 50.; //R matrix variance for position
const float acc_measurement_variance = 10.; //R matrix variance for acceleration
VectorXf kalmanState = VectorXf::Zero(x_dim);    // KF state vector, updated with get_x()
float alpha = 0.01, beta = 2.0, kappa = 0.;  //Sigma points generation parameters using the Merwe Method

//Rocket propertiesm used for the prediction step of the KalmanFilter
const double gravity = -9.8;  // m/s²
const double Cd = 0.1;        // Drag coefficient
const double rho = 1.225;     // Air density (kg/m³)
const double A = 0.01;        // Cross-sectional area (m²)
const double mass = 5.0;      // Mass (kg)


//Simulation variables
const bool IS_SIMULATION = true;  //This variable allows for software on a loop integration
const bool VERBOSE = true;        //This variable allows for explicit printing of stage change and other flight events for debugging
int simDataColumnNumber = 0; //Number of columns in the CSV simulation data file
float baroStdDev = 5; // Expected noise of the barometer sensor
float accelStdDev = 2; //Expected noise of the accelerometer sensor


//Declare managers:
PyroController pyro;
GPSController gps;
MemoryManager flash;
MemoryManager sd;
SensorManager sens;
LoRaComm lora;
MerweScaledSigmaPoints sigmaPoints(x_dim, alpha, beta, kappa);
KalmanFilter kf(x_dim, z_dim, u_dim, sigmaPoints);


STAGES currentStage; //The stages are handled using an enumerator in Constants.h

void setup() {
  Serial.begin(9600);
  pinMode(RLED, OUTPUT); //Cause why not
  currentStage = STARTUP;
}

void loop() {
  //Outside the switch structure is everything that runs in every single iteration no matter the rocket stage
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

/*
  * Samples data from the rocket enviroment or it pulls data from the SD card if it is a Simulation
*/
void sample() {
  static float prevSampleTime = 0.0f;
  if (IS_SIMULATION) {
    currentData["intTime"] = 0;
    while(currentData["intTime"] < (sampleDelay/1000.)) { //This while loop is so we can manage slowed-reading rates for KF performance and still have a high resolution data file
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
  if (reset) messCounter = 0; //allows to reset the index
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
  * Every termination function terminates a stage, meaning, it detects that the stage termination condiction was matched to 
  asign the current stage as the next one in order.
*/

void startUpInit() {
  /*  
    Initialize board's neccesary systems
    It only works the first time it is ever called so sensors are not accidently rebooted
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
    if (!flash.begin('f', CS_FLASH)) { //f for flash
      while (1) {
        if (VERBOSE) {
          Serial.println("FLASH NOT FOUND");
        }
        delay(1000);
      }
    }
    if (!sd.begin('s', CS_SD)) { //s for SDcard
      while (1) {
        if (VERBOSE) {
          Serial.println("COULDN'T REACH SD CARD");
        }
        delay(1000);
      }
    }
    if (IS_SIMULATION) { //Initialize simulation if we are running one
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
      * Initialize Kalman Filter for state tracking.
    */
    VectorXf initial_state(x_dim);
    MatrixXf initial_covariance(x_dim, x_dim);
    initial_state << 0, 0, 0;
    initial_covariance << 500, 0, 0, //could be all zeros as we know for sure that the rocket is in the ground and idleing
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
    /*
      We can't use the Fusion mode of the BNO055 as it's maximum acceleration is 4g's and the rocket is expected
      to feel up to 6g on launch
    */
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
  /*
    Check for stage finalization condition:
    * The altitude is lower than 1200mts
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