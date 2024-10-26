#ifndef LORACOMM_H
#define LORACOMM_H

#include <LoRa.h>

class LoRaComm {
  public:
    LoRaComm();
    bool begin();
    void transmitData(float dataList[], int dataID);
  private:
    byte rocketAddress = 0b01010101;
};


#endif
