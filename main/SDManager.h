#ifndef SDMANAGER_H
#define SDMANAGER_H
#include <SD.h>


class SDManager {
  public:
    File dataFile;
    SDManager();
    bool begin();
    void logData(float message[]);
};

#endif

