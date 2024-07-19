#include "SensorConfig.h"

void setup() {
  
  Serial.begin(115200);

  bmeActivation();

  float bmeVariables[4] = {};

  bnoActivation();

  xtsdActivation();

  loraActivation();

  gpsActivation();

}

void loop() {

  bmeVariables[0] =  bme.readTemperature(); //°C
  bmeVariables[1] =  bme.readPressure()/100.0F ; //hPa
  bmeVariables[2] =  bme.readAltitude(SEALEVELPRESSURE_HPA); //m
  bmeVariables[3] =  bme.readHumidity(); //%
  
  

}
