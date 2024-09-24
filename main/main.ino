#include "SensorManager.h"
#include "PyroController.h"
#include "LoRaComm.h"
#include "SDManager.h"
#include "GPSController.h"


float message[18] = {};

PyroController pyro;
GPSController gps;
SDManager sd;
SensorManager sensor;

void setup() {
  Serial.begin(115200);
  sd.begin();
  sensor.begin();
  pyro.begin();
  gps.begin();
}

void loop() {}
