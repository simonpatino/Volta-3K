#ifndef LORACOMM_H
#define LORACOMM_H

#include <LoRa.h>

class LoRaComm {
  public:
    LoRaComm();
    bool begin();
    void transmitData(float bmeVariables[], float bnoVariables[],
                      bool pyroVariables[], long gpsVariables[], 
                      float message[]);
};


#endif
