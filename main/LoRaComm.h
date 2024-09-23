#ifndef LORACOMM_H
#define LORACOMM_H

void setupLoRa();
void transmitData(float bmeVariables[], float bnoVariables[],
                  bool pyroVariables[],float  gpsVariables[], 
                  float message[]);

#endif
