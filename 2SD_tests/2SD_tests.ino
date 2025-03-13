#include <SD.h>

// Define CS pins for external SD cards
const int csPin1 = 2; // CS pin for external SD card 1

void setup() {
  // Initialize the internal SD card (SDIO)
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Internal SD card initialization failed!");
    return;
  }
  Serial.println("Internal SD card initialized.");

  // Initialize external SD card 1 (SPI)
  if (!SD.begin(csPin1)) {
    Serial.println("External SD card 1 initialization failed!");
    return;
  }
  Serial.println("External SD card 1 initialized.");
}

void loop() {
  // Open files on both SD cards
  File fileInternal = SD.open("log.txt", FILE_WRITE);
  File fileExternal1 = SD.open("log1.txt", FILE_WRITE);

  if (fileInternal && fileExternal1) {
    // Write data to both files
    String data = "Log data: " + String(millis());
    fileInternal.println(data);
    fileExternal1.println(data);

    // Close files
    fileInternal.close();
    fileExternal1.close();
    Serial.println("Wrote");
  } else {
    Serial.println("Error opening files!");
  }

  delay(1000); // Wait before writing again
}