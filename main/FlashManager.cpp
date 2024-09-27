#include <Arduino.h>
#include "FlashManager.h"
#include <SD.h>
#include "Constants.h"

FlashManager::FlashManager() {
}

bool FlashManager::begin() {
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13); 
  
  if (!SD.begin(CS_FLASH)) {
    return 0;
  } else {
    return 1;
  }
}

void FlashManager::logData(float message[]) {
 dataFile = SD.open("Volta.txt", FILE_WRITE);
    if (dataFile) {
        for (int i = 0; i < 18; i++) {
            dataFile.print(message[i]);
            if (i < 17) dataFile.print(", ");
        }
        dataFile.println();
        dataFile.close();
    } else {
        Serial.println("Error opening Volta.txt");
    }
}
