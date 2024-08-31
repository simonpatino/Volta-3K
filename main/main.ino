#include "SensorConfig.h"

float bmeVariables[4] = {};
float bnoVariables[3] = {};
float message[7] = {};

long pyro_timer;

void setup() {
  
  Serial.begin(115200);

  checkActivation();

  bmeActivation();

  bnoActivation();

  xtsdActivation();

  //loraActivation();

  //gpsActivation();

/*   ejectEvent(PYRO1_A);
  //ejectEvent(PYRO1_B);

  ejectEvent(PYRO2_A);
  //ejectEvent(PYRO2_B);

  ejectEvent(PYRO3_A);
  //ejectEvent(PYRO3_B);

  ejectEvent(PYRO4_A);
  //ejectEvent(PYRO4_B);

  ejectEvent(PYRO5_A);
  //ejectEvent(PYRO5_B); */

  analogWrite(R , 255);
  analogWrite(G , 255);
  analogWrite(B , 255);

}

void loop() {

bmeVariables[0] = bme.readTemperature();                   //°C
bmeVariables[1] = bme.readPressure() / 100.0F;             //hPa
bmeVariables[2] = bme.readAltitude(SEALEVELPRESSURE_HPA);  //m
bmeVariables[3] = bme.readHumidity();                      //%

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

bnoVariables[0] = euler.x();                                      //degree
                  bnoVariables[1] = euler.y();                    //degree
                                    bnoVariables[2] = euler.z();  //degree

message[0] = bmeVariables[0];
message[1] = bmeVariables[1];
message[2] = bmeVariables[2];
message[3] = bmeVariables[3];
message[4] = bnoVariables[0];
message[5] = bnoVariables[1];
message[6] = bnoVariables[2];


dataFile = SD.open("Volta.txt", FILE_WRITE);

  if (dataFile) {
    // Write the message array to the file in a single line
    for (int i = 0; i < 7; i++) {
      dataFile.print(message[i]);
      if (i < 6) {
        dataFile.print(", ");  // Add a comma and space between the numbers
      }
    }
    dataFile.println(); // Move to the next line after writing all data
    dataFile.close();   // Close the file
  } else {
    Serial.println("Error opening data.txt");
  }

  Serial.print("| Ti: ");
  Serial.print(millis()/1000);
  Serial.print(" | X: ");
  Serial.print(euler.x());
  Serial.print(" | Y: ");
  Serial.print(euler.y());
  Serial.print(" | Z: ");
  Serial.print(euler.z());
  Serial.print(" | Te: ");
  Serial.print(bmeVariables[0]); 
  Serial.print(" | P: ");
  Serial.print(bmeVariables[1]); 
  Serial.print(" | ALT: ");
  Serial.print(bmeVariables[2]); 
  Serial.print(" | HUM: ");
  Serial.print(bmeVariables[3]); 
  Serial.print(" | PYRO1A: ");
  pyroCheck(PYRO_CHECKS[0][0]); 
  Serial.println("");

  delay(500);

  
  if(millis() > 10000 && millis() < 10800) {
    //firePyro(0,'a');
    pyro_timer = millis();
  } else {
    killPyros();
  }

  
}
