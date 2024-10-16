#ifndef GPS_H
#define GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>

class GPSController {
  public:
    GPSController();
    bool begin();
    bool checkGPS();
    SFE_UBLOX_GPS myGPS;
    long gpsVariables[2] = {}; 
};

#endif
