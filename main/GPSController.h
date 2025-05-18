#ifndef GPS_H
#define GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>

class GPSController {
  public:
    bool verbose;
    GPSController();
    bool begin();
    bool updateGPS(bool verbose = true);
    int getFixes();
    float getLatitude();
    float getLongitude();
    float getVel();
    float getAltitude();
  private:
    float latitude, longitude, vel;
};

#endif
