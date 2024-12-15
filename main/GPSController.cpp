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


bool GPSController::updateGPS() {
    myGPS.checkUblox();
    if (nmea.isValid()) {
      latitude = nmea.getLatitude() / 1000000.0;
      longitude = nmea.getLongitude() / 1000000.0;
      return 1;
    } else {
      Serial.print("No Fix, Num satellites: ");
      Serial.println(nmea.getNumSatellites());
      return 0;
    }
}

float GPSController::getLatitude() {
  return latitude;
}

float GPSController::getLongitude() {
  return longitude;
}
int GPSController::getFixes() {
  return nmea.getNumSatellites();
}