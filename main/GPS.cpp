#include "GPS.h"
#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>

SFE_UBLOX_GPS myGPS;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void gpsActivation() {

  Wire2.begin();

  if (myGPS.begin(Wire2) == false) {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }
}

void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}


void checkGPS(double gpsVariables[]) {
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
    } else {
        Serial.print("No Fix, Num satellites: ");
        Serial.println(nmea.getNumSatellites());
    }
}
