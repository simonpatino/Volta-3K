#include "SensorConfig.h"

float bmeVariables[4] = {};


void setup() {
  
  Serial.begin(115200);

  bmeActivation();

  //bnoActivation();

  //xtsdActivation();

  //loraActivation();

  //gpsActivation();

}

void loop() {

  bmeVariables[0] =  bme.readTemperature(); //°C
  bmeVariables[1] =  bme.readPressure()/100.0F ; //hPa
  bmeVariables[2] =  bme.readAltitude(SEALEVELPRESSURE_HPA); //m
  bmeVariables[3] =  bme.readHumidity(); //%
  
}
