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


bool GPSController::updateGPS(bool verbose /* = true */) { // Update signature, comment shows default
  // Serial.println("GPSController: Checking Ublox module..."); // Removed as requested

  myGPS.checkUblox(); // Check for new data from the module

  int satellites = myGPS.getSIV(); // Get number of satellites in view
  if (verbose) {
    Serial.print("GPSController: Satellites in view: "); // Debug: Report satellites found
    Serial.println(satellites);
  }

  if (satellites >= 3) {
    latitude = myGPS.getLatitude() / 1000000.0;
    longitude = myGPS.getLongitude() / 1000000.0;
    float v_n = myGPS.getNedNorthVel() / 1000.0; // Convert mm/s to m/s
    float v_e = myGPS.getNedEastVel() / 1000.0;   // Convert mm/s to m/s
    float v_d = myGPS.getNedDownVel() / 1000.0;   // Convert mm/s to m/s
    vel = sqrt(v_n * v_n + v_e * v_e + v_d * v_d);
    
    if (verbose) {
      Serial.print("GPSController: GPS Fix obtained. Satellites: "); // Debug: Fix obtained
      Serial.print(satellites);
      Serial.print(" Lat: ");
      Serial.print(latitude, 6);
      Serial.print(" Lon: ");
      Serial.println(longitude, 6);
    }
    return true; // Indicate successful update with fix
  } else {
    // No fix or insufficient satellites
    if (verbose) {
      Serial.print("GPSController: No GPS Fix or insufficient satellites. Satellites found: "); // Debug: No fix, show satellite count
      Serial.println(satellites);
    }
    // Optionally reset latitude/longitude or set to specific invalid values
    // latitude = 0.0; 
    // longitude = 0.0;
    return false; // Indicate update failed or no fix
  }
}

float GPSController::getLatitude() {
  return latitude;
}

float GPSController::getLongitude() {
  return longitude;
}

float GPSController::getVel() {
  return vel;
}

int GPSController::getFixes() {
  return myGPS.getSIV();
}