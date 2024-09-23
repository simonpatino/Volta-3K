#include "SensorConfig.h"
#include "Pyro.h"
#include "LoRaComm.h"
#include "SDManager.h"
#include "GPS.h"

float bmeVariables[4] = {};
float bnoVariables[3] = {};
float pyroVariables[8] = {};
float gpsVariables[2] = {}; 
float message[18] = {};

void setup() {
  Serial.begin(115200);
  
  setupSensors();
  setupLoRa();
  setupPyro();
  setupSD();
  setupGPS();
}

void loop() {
  readSensors(bmeVariables, bnoVariables);
  checkPyro();
  checkGPS();
  transmitData(bmeVariables, bnoVariables, message);
  logData(message);
}
