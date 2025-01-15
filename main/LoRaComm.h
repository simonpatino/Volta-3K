#ifndef LORACOMM_H
#define LORACOMM_H

#include <LoRa.h>

class LoRaComm {
  public:
    LoRaComm();
    bool begin();
    void transmitData(float dataList[], int length, byte dataID);
    void transmitGPS(int satNum, float lat, float lon);
    void transmitPyroInfo(bool pyroContinuity[10]);
    void checkReceive();
    static byte lastCommand;
    static void onReceive(int packetSize);
  private:
    static void transmitConfirmation();
    static const byte confirmationCode = 0x07;
    const byte gpsDataID = 0x03;
    const byte pyroDataID = 0x04;
};


#endif
