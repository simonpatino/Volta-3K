/*
  TODO:
  Tasks that we depend on Simon for:
  - Include the PID test algorithm, log its output
  - Enhance KF fx function to reduce process variance, figure out how OpenRocket does it (maybe a RungeKutta?) | Simon please help imma die her
*/

#include <Arduino.h>
#include "Sigma.h"           //For creating and managing the sigmapoints used in the UKF
#include "KalmanFilter.h"    //For managing the KF atributes and equations
#include "SensorManager.h"   //For managing the BNO055 and the BME280
#include "PyroController.h"  // Manages the pyrochannels check, fire and kill capabilites
#include "LoRaComm.h"        // Manages the LoRa module
#include "MemoryManager.h"   // Manages both the Flash module and the Teensy 4.1 built-in microSD card slot
#include "GPSController.h"   // Manages the GPS M9N
#include <map>               // The last sensed data is saved in a map so it is easier  to access the data by all code sectors that might need it
#include <ArduinoEigen.h>    // To interact with the Kalman Filter using matrices and vectors
#include <string>
#include "Functions.h"


const int messCoreLenght = 15;  //Lenght of the main data packet that we'll save in memory and send through LoRa
const int mess2Lenght = 5;      //Lenght of the secundary data packet that we'll save in

//Current order: "Iteration, Time (s), Accel_0, Accel_1, Accel_2, Alt, Press, Euler0, Euler1, Euler2, MaxAlt, Stage #, Sat #, Latitude, Longitude"
float messageCore[messCoreLenght] = {};  //The actual data in a float array. It has an specific order.

//Current order: Temperature, Humidity, Gyro X, Gyro Y, Gyro Z
float messageSecundary[mess2Lenght] = {};  //The actual data in a float array. It has an specific order

std::map<String, float> currentData;  // The last recorded data
/*
  KEYS
  - iter: the number of the iteration of sensed data, from 1 and on
  - time: current time
  - temp: sensed temperature by BME280
  - prss: sensed pressure by BME280
  - alt: the sensed (then noisy) altitude of the rocket given by the BME280
  - deltaAlt: difference between the last sensed alt and the current alt. Should not be used and maybe should be deleted.
  - humty: humidity
  - angVelData0: X-axis gryscope reading
  - angVelData1: Y-axis gryscope reading
  - angVelData2: Z-axis gryscope reading
  - accData0: X-axis acceleration reading
  - accData1: Y-axis acceleration reading
  - accData2: Z-axis acceleration reading of the BNO055 (with noise)
    The next 6 variables can only be read if the BNO055 is set to be in Fusion Mode (using the built-in Kalman Filter)
  - euler0: X-axis rotation
  - euler1: Y-axis rotation
  - euler2: Z-axis rotation
  - linAccData0: intertial-X-axis acceleration
  - linAccData1: intertial-Y-axis acceleration
  - linAccData2: intertial-Z-axis acceleration
  - maxAlt: maximum altitude reached by the rocket
  - stage: the current stage of the rocket
  - sat: the number of the satelite that is being tracked by the GPS
  - lat: latitude
  - lon: longitude

  For simulation only:
  - rawVel: velocity reading from the simulation, used to verify KF performance

*/

//Global variables that might be useful here and there.
int sampleDelay = 500;                                                                                //Fixed sampling delay for the whole system (in ms)
long lastTime = millis();                                                                             //Current time since boot up. Manages the dynamic delay
long cycleNumber = 0;                                                                                 //Cycle number counter. Works as an ID for each data package
bool continuityPyros[10] = { false, false, false, false, false, false, false, false, false, false };  //Array with the continuity data of the pyrochannels
float powderChambTemp[4] = {};                                                                        // Array with temperature data from the LM35


//KalmanFilter variables
const int x_dim = 3;
const int z_dim = 2;
const int u_dim = 0;
const float process_variance = 30.0f;          //Q variance for set_Q_discrete_white_noise()
const float pos_measurement_variance = 20.0f;  //R matrix variance for position
const float acc_measurement_variance = 10.0f;  //R matrix variance for acceleration
VectorXf kalmanState = VectorXf::Zero(x_dim);  // KF state vector, updated with get_x()
int kfIteration = 0;                           //Record of how many KFSteps have happened.
float alpha = 0.01, beta = 2.0, kappa = 0.;    //Sigma points generation parameters using the Merwe Method

//Rocket properties used for the prediction step of the KalmanFilter. Pulled from the OpenRocket simulation
const float gravity = -9.81;             // m/s²
float const Cd_rocket = 0.47;            // Drag coefficient
float const Cd_parachute_drogue = 1.16;  // Drag coefficient
float const Cd_parachute_main = 2.92;    // Drag coefficient
float Cd = Cd_rocket;                    // Drag coefficient
const float rho = 1.112;                 // Air density (kg/m³)
const float A = 0.023787;                // Cross-sectional area (m²)
const float mass = 30.747;               // Mass (kg)

bool trustKalman = false;


//Simulation variables
const bool IS_SIMULATION = false;  //This variable allows for software on a loop integration
const bool VERBOSE = true;         //This variable allows for explicit printing of stage change and other flight events for debugging
int simDataColumnNumber = 0;       //Number of columns in the CSV simulation data file
float baroStdDev = 20.0f;          // Expected noise of the barometer sensor
float accelStdDev = 10.0f;         //Expected noise of the accelerometer sensor

//Flight managment variables
bool groundConfirmation = false;
uint32_t sleepinterval = 30000; //Timer to 30 seconds


//Declare managers:
PyroController pyro;
GPSController gps;
MemoryManager mem;
SensorManager sens;
LoRaComm lora;
MerweScaledSigmaPoints sigmaPoints(x_dim, alpha, beta, kappa);
KalmanFilter kf(x_dim, z_dim, u_dim, sigmaPoints);


STAGES currentStage;  //The stages are handled using an enumerator in Constants.h

const bool manualCalibration = false;


bool KEY = true;
File myFile;
const int chipSelect = 2;  // Adjust based on your setup
//#####################################################



void setup() {
  Serial.begin(9600);
  pinMode(RLED, OUTPUT);  //Cause why not
  pinMode(BLED, OUTPUT);  //Cause why not
  pinMode(GLED, OUTPUT);  //Cause why not

  currentStage = STARTUP;


  Serial.println("********************************************");
  Serial.println("* Write Something to Enter in WRITER MODE  *");
  Serial.println("********************************************");



  //NORMAL OR WRITER MODE 
  float STAR_TIME = 10.0f;  
  while ( STAR_TIME*1000 - millis() > 0){
    if (Serial.available()){
      KEY = false;

    digitalWrite(RLED, HIGH);  // White color, why not
    digitalWrite(BLED, HIGH);  // White color, why not
    digitalWrite(GLED, HIGH);  // White color, why not

    Serial.println("██     ██ ██████  ██ ████████ ███████ ██████  ");
    Serial.println("██     ██ ██   ██ ██    ██    ██      ██   ██ ");
    Serial.println("██  █  ██ ██████  ██    ██    █████   ██████  ");
    Serial.println("██ ███ ██ ██   ██ ██    ██    ██      ██   ██ ");
    Serial.println(" ███ ███  ██   ██ ██    ██    ███████ ██   ██ ");


    Serial.println("**********************************************");
    Serial.println("*++++++++++++ Wait a Few Seconds *************");
    Serial.println("**********************************************");

    delay(5000);

    break;
    } 
    
  }



}

//#####################################################

    void loop() {

      if (KEY){
      //Outside the switch structure is everything that runs in every single iteration no matter the rocket stage
      dynamicDelay();
      //pyro.readBayTempAll(powderChambTemp);
        switch (currentStage) {
          case STARTUP:
            startUpInit();
            gps.updateGPS();
            checkCommand();
            startupTermination();
            
            break;
          case IDLE:
            idleInit();
            /*
              More wanted actions here
            */
            checkCommand(); //check if there is a command to be executed
            sample();
            parseData();
            //serialPrintMessage();
            mem.logFloatData(messageCore, messCoreLenght, mem.dataFileName, true);
            mem.logBoolData(continuityPyros, currentData["time"], 10, mem.pyroFileName);
            transmitDataDelayed();

            idleTermination();
            break;

          case BOOSTING:
            boostInit();
            /*
            More Boosting actions here
            */
            sample();
            parseData();
            transmitDataDelayed();
            serialPrintMessage();
            mem.logFloatData(messageCore, messCoreLenght, mem.dataFileName, true);
            boostTermination();
            break;

          case COASTING:
            coastInit();
            /*
            More wanted actions here
            */
            KFStep();
            sample();
            parseData();
            transmitDataDelayed();
            serialPrintMessage();
            mem.logFloatData(messageCore, messCoreLenght, mem.dataFileName, true);
            coastTermination();
            break;

          case DROGUEDESCENT:
            drogueInit();
            /*
            More wanted actions here
            */
            //KFStep();
            sample();
            parseData();
            transmitDataDelayed();
            serialPrintMessage();
            mem.logFloatData(messageCore, messCoreLenght, mem.dataFileName, true);
            drogueTermination();
            break;

          case MAINDESCENT:
            mainDescentInit();
            /*
            More wanted actions here
            */
            sample();
            parseData();
            transmitDataDelayed();
            serialPrintMessage();
            mem.logFloatData(messageCore, messCoreLenght, mem.dataFileName, true);
            mainDescentTermination();
            break;

          case TOUCHDOWN:

            touchDownInit();
            sample();
            parseData();
            transmitDataDelayed();
            serialPrintMessage();
            
            mem.logFloatData(messageCore, messCoreLenght, mem.dataFileName, true);
            break;

          // Add more cases as needed
          default:
            // Code to execute if none of the above cases match

            Serial.println("Unknown stage. Please check the code.");
            
            break;
        }

      } else {

        // CODE WRITER 


        Serial.println("");
        Serial.print("Initializing SD card...");
        if (!SD.begin(chipSelect)) {
          Serial.println("Initialization failed!");
          return;
        }
          Serial.println("Initialization done.");


        Serial.println("***************************");
        Serial.println("* Write 'R' for Volta.txt *");
        Serial.println("* Write 'P' for pyro.txt *");
        Serial.println("***************************");

        while (Serial.available() > 0) {  // Clear any existing data
        Serial.read();                    // Read and discard
        }

        while( Serial.available() == 0){
          ;
        }
          
        if (Serial.available() > 0) {
          char command = Serial.read();
          if (command == 'R') {
            sendFileContent("Volta.txt");

            while (true){
              ;             //Do Nothing Forever
            }

          } else if (command == 'P') {
            sendFileContent("pyro.txt");

            while(true) {
              ;             //Do nothing Forever 
            }  

            }
        }

      }

/////////
    }


