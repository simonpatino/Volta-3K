#include <SD.h>
#include <SPI.h>

File myFile;
const int chipSelect = 2;  // Adjust based on your setup

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // Wait for serial port to connect.
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'R') {
      sendFileContent("Volta.txt");
    } else if (command == 'P') {
      sendFileContent("pyro.txt");  
    }
  }
}

void sendFileContent(const char* filename) {
  myFile = SD.open(filename);
  if (myFile) {
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.print("Error opening ");
    Serial.println(filename);
  }
}
