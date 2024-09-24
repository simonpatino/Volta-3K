#ifndef GPS_H
#define GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>

class GPSController {
  public:
    GPSController();
    bool begin();
    void checkGPS();
    SFE_UBLOX_GPS myGPS;
    char nmeaBuffer[100];
    MicroNMEA nmea;
    long gpsVariables[2] = {}; 
};

#endif
