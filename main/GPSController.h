#ifndef GPS_H
#define GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>

class GPSController {
  public:
    GPSController();
    bool begin();
    bool updateGPS();
    int getFixes();
    float getLatitude();
    float getLongitude();
    SFE_UBLOX_GPS myGPS;
  private:
    float latitude, longitude;
};

#endif
