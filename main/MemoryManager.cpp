#include <Arduino.h>
#include "MemoryManager.h"
#include <SD.h>
#include "Constants.h"
#include "SPI.h"



MemoryManager::MemoryManager() {
}

bool MemoryManager::begin(char type, int csPin) {
  if(type == 'f') {
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13); 
  }
  
  return SD.begin(csPin);
}

void MemoryManager::logData(float message[]) {
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

void MemoryManager::pullData() {

}
