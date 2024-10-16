#include <Arduino.h>
#include "GPSController.h"
#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>  

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

GPSController::GPSController() {}

bool GPSController::begin() {
  Wire2.begin();
  if (myGPS.begin(Wire2) == false) {
    return 0;
  }
  return 1;
}

void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  nmea.process(incoming);
}


bool GPSController::checkGPS() {
    myGPS.checkUblox();
    if (nmea.isValid()) {
        long latitude = nmea.getLatitude();
        long longitude = nmea.getLongitude();
        gpsVariables[0] = latitude;
        gpsVariables[1] = longitude;
        Serial.print("Latitude: ");
        Serial.println(latitude / 1000000.0, 6);
        Serial.print("Longitude: ");
        Serial.println(longitude / 1000000.0, 6);
        return 1;
    } else {
        Serial.print("No Fix, Num satellites: ");
        Serial.println(nmea.getNumSatellites());
        return 0;
    }
}
