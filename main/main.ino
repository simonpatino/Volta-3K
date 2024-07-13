#include "SensorConfig.h"

void setup() {
  
  Serial.begin(9600);

  bmeActivation();

  bnoActivation();

  //xtsdActivation();

}

void loop() {
  

}
