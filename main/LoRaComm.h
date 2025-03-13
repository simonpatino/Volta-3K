#ifndef LORACOMM_H
#define LORACOMM_H

#include <LoRa.h>

class LoRaComm {
  public:
    LoRaComm();
    bool begin();
    void transmitData(float dataList[], int length);
    void transmitStageChange(int stage);
    void transmitPyroInfo(bool pyroContinuity[10]);
    void transmitString(String message);
    void checkReceive();
    void setToSleep();
    void wakeUp();
    static byte lastCommand;
    static void onReceive(int packetSize);
  private:
    static void transmitConfirmation();
    static const byte coreDataID = 0x00;
    static const byte stringID = 0x02;
    static const byte gpsDataID = 0x03;
    static const byte pyroDataID = 0x04;
    static const byte stageDataID = 0x06;
    static const byte confirmationCode = 0x07;
};


#endif
