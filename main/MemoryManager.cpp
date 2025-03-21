#include <Arduino.h>
#include "MemoryManager.h"
#include <SD.h>
#include "Constants.h"
#include "SPI.h"
#include <map>


MemoryManager::MemoryManager() {
}

bool MemoryManager::begin(int csPin) {
  pinMode(csPin, OUTPUT);
  bool b = SD.begin(csPin);
  digitalWrite(CS_SD, HIGH);
  return b;
}

void MemoryManager::logFloatData(float message[], int length, const char* fileName, bool withGPS) {
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    if(withGPS) {
      for (int i = 0; i < length-2; i++) {
        dataFile.print(message[i], 3);
        dataFile.print(", ");
      }
      dataFile.print(message[length-2], 7);
      dataFile.print(", ");
      dataFile.print(message[length-1], 7);
    } else {
      for (int i = 0; i < length; i++) {
        dataFile.print(message[i], 3);
        dataFile.print(", ");
      }
    }
    dataFile.println();
    dataFile.close();
  }
}

void MemoryManager::logBoolData(bool message[], float time, int length, const char* fileName) {
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(time, 2);
    for (int i = 0; i < length; i++) {
      dataFile.print(message[i]);
      dataFile.print(", ");
    }
    dataFile.println();
    dataFile.close();
  }
}

// Function to detect the number of columns in the CSV file
int MemoryManager::startSimulationData() {
  digitalWrite(CS_SD, HIGH);
  digitalWrite(CS_FLASH, LOW);
  simFile = SD.open(simFileName, FILE_READ);
  digitalWrite(CS_SD, LOW);
  if (!simFile) {
    Serial.println("Error opening simulation file.");
    while (1) {
      delay(500);
      Serial.println("Killed");
    }
    return 0;
  }

  if (!simFile.available()) {
    return -1;  // Error: file is empty or not accessible
  }

  String firstLine = simFile.readStringUntil('\n');  // Read the first line of the file

  for (unsigned int i = 0; i < firstLine.length(); i++) {
    if (firstLine[i] == ',') {
      numColumns++;  // Increment for each comma found
    }
  }

  //simFile.seek(0);  // Reset the file pointer to the beginning of the file IF there is no header in the csv file
  return numColumns;
}

/*
  Inputs:
    - An empty array of size n
  
  Considerations
    - The CSV file need to have information in the following order:
    Iteration, Time, Acc X (m/s²), Acc Y (m/s²), Avv Z (m/s²),	Altitude (m),	Air pressure (mbar/hPa),	
    Roll rate (°/s),	Pitch rate (°/s),	Yaw rate (°/s), Latitude (°),	Longitude (°)

*/
bool MemoryManager::readSimulatedData(std::map<String, float>& dataDict) {
  float rowData[numColumns];
  if (!simFile.available()) {
    return false;  // No more data to read because the pointer reached the end of the file
  }

  String line = simFile.readStringUntil('\n');  // Read a line from the file and move the internal pointer to the next line
  int startIndex = 0;

  for (int column = 0; column < numColumns; column++) {
    int separatorIndex = line.indexOf(',', startIndex);
    String cell;

    if (separatorIndex == -1) {  // Last value in the row, no more commas found
      cell = line.substring(startIndex);
    } else {
      cell = line.substring(startIndex, separatorIndex);
      startIndex = separatorIndex + 1;
    }

    rowData[column] = cell.toFloat();  // Convert the string to float and store it
  }
  //Serial.println(numColumns);
  /*   for (int i = 0; i < numColumns; i++) {
    Serial.print(rowData[i]);
    Serial.print(", ");
  }
  Serial.println(); */
  static float prevAtl;
  dataDict["iter"] = rowData[0];
  dataDict["time"] = rowData[1];
  dataDict["temp"] = rowData[12];    // °C
  dataDict["prss"] = rowData[6];     // hPa
  dataDict["realAlt"] = rowData[5];  // hPa
  dataDict["deltaAlt"] = dataDict["alt"] - prevAtl;
  dataDict["humty"] = rowData[13];  // %;
  prevAtl = dataDict["alt"];
  if (dataDict["alt"] > dataDict["maxAlt"]) dataDict["maxAlt"] = dataDict["alt"];
  dataDict["euler0"] = 0.0;
  dataDict["euler1"] = 0.0;
  dataDict["euler2"] = 0.0;

  dataDict["accData0"] = rowData[2];
  dataDict["accData2"] = rowData[3];

  dataDict["linAccData0"] = rowData[2];
  dataDict["linAccData2"] = rowData[3];

  dataDict["realAccData1"] = rowData[4];
  
  dataDict["angVelData0"] = rowData[9];
  dataDict["angVelData1"] = rowData[8];
  dataDict["angVelData2"] = rowData[7];


  dataDict["rawVel"] = rowData[numColumns - 1];  //Raw (noisy) velocity from the simulation, is the last column

  return true;  // Successfully read the row
}

void MemoryManager::checkAndDeleteFile(const char* filename) {
  if (SD.exists(filename)) {    // Check if the file exists
    if (SD.remove(filename)) {  // Try to delete the file
      Serial.print("File deleted successfully: ");
      Serial.println(filename);
    }
  }
}

void MemoryManager::deleteFiles() {
  checkAndDeleteFile(dataFileName);
  checkAndDeleteFile(pyroFileName);
  checkAndDeleteFile(kfRealFileName);
  checkAndDeleteFile(kfMeasuFileName);
  checkAndDeleteFile(kfConfigFileName);
  checkAndDeleteFile(kfOutputFileName);
  checkAndDeleteFile(kfPerformanceFileName);
}
