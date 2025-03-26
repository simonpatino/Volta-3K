#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <map>
#include <string>
#include "Sigma.h"
#include "KalmanFilter.h"
#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "MemoryManager.h"
#include "GPSController.h"

// Forward declarations


// Global variable declarations (extern)
extern const int messCoreLenght;
extern const int mess2Lenght;
extern float messageCore[];
extern float messageSecundary[];
extern std::map<String, float> currentData;
extern int sampleDelay;
extern long lastTime;
extern long cycleNumber;
extern bool continuityPyros[10];
extern float powderChambTemp[4];
extern const int x_dim;
extern const int z_dim;
extern const float process_variance;
extern const float pos_measurement_variance;
extern const float acc_measurement_variance;
extern VectorXf kalmanState;
extern int kfIteration;
extern const float gravity;
extern float Cd;
extern const float Cd_rocket;
extern const float Cd_parachute_drogue;
extern const float Cd_parachute_main;
extern const float rho;
extern const float A;
extern const float mass;
extern const bool IS_SIMULATION;
extern const bool VERBOSE;
extern int simDataColumnNumber;
extern float baroStdDev;
extern float accelStdDev;
extern bool groundConfirmation;
extern uint32_t sleepinterval;
extern STAGES currentStage;
extern const bool manualCalibration;
extern bool trustKalman;


// Manager objects
extern PyroController pyro;
extern GPSController gps;
extern MemoryManager mem;
extern SensorManager sens;
extern LoRaComm lora;
extern MerweScaledSigmaPoints sigmaPoints;
extern KalmanFilter kf;

// Function declarations
void serialPrintMessage();
void sample();
float addGaussianNoise(float value, float mean, float stdDev);
void KFStep();
VectorXf fx(const VectorXf& x, float dt);
VectorXf hx(const VectorXf& x);
void messageAppend(float info, bool reset = false);
void parseData();
void dynamicDelay();
void sendFileContent(const char* filename);

// Flight stage functions
void startUpInit();
void startupTermination();
void idleInit();
void idleTermination();
void boostInit();
void boostTermination();
void coastInit();
void coastTermination();
void drogueInit();
void drogueTermination();
void mainDescentInit();
void mainDescentTermination();
void touchDownInit();
void checkCommand();
void transmitDataDelayed();

#endif
