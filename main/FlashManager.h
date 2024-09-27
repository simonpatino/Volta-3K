#ifndef FLASHMANAGER_H
#define FLASHMANAGER_H
#include <SD.h>

class FlashManager {
  public:
    File dataFile;
    FlashManager();
    bool begin();
    void logData(float message[]);
};

#endif

