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
  private:
    float latitude, longitude;
};

#endif
