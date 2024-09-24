#include <Arduino.h>
#include "SDManager.h"
#include <SD.h>
#include "Constants.h"

SDManager::SDManager() {
}

bool SDManager::begin() {
  SPI.setMOSI(11);  // Audio shield has MOSI on pin 7
  SPI.setMISO(12);  // Audio shield has MOSI on pin 7
  SPI.setSCK(13);   // Audio shield has SCK on pin 14

  if (!SD.begin(CS_FLASH)) {
    return 0;
  } else {
    return 1;
  }
}

void SDManager::logData(float message[]) {
/*     dataFile = SD.open("Volta.txt", FILE_WRITE);
    if (dataFile) {
        for (int i = 0; i < 18; i++) {
            dataFile.print(message[i]);
            if (i < 17) dataFile.print(", ");
        }
        dataFile.println();
        dataFile.close();
    } else {
        Serial.println("Error opening Volta.txt");
    } */
}
