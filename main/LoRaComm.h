#ifndef LORACOMM_H
#define LORACOMM_H

void setupLoRa();
void transmitData(float bmeVariables[], float bnoVariables[],
                  float pyroVariables[],long  gpsVariables[], 
                  float message[]);

#endif
