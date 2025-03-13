#include <Arduino.h>
#include "GPSController.h"
#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;

GPSController::GPSController() {}

bool GPSController::begin() {
  Wire2.begin();
  if (myGPS.begin(Wire2) == false) {
    return 0;
  }
  myGPS.setI2COutput(COM_TYPE_UBX);  //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();         //Save the current settings to flash and BBR
  return 1;
}


bool GPSController::updateGPS() {
  /*
    * If it ain't broke don't try to fix it
  */
  myGPS.checkUblox();
  if (myGPS.getSIV() > 4) {
    latitude = myGPS.getLatitude() / 1000000.0;
    longitude = myGPS.getLongitude() / 1000000.0;
    return 1;
  } else {
    Serial.print("No Fix, Num satellites: ");
    Serial.println(myGPS.getSIV());
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
  return myGPS.getSIV();
}