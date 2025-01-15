#ifndef MEMORYMANAGER_H
#define MEMORYMANAGER_H
#include <SD.h>

class MemoryManager {
  public:
    File dataFile;
    MemoryManager();
    bool begin(char type, int csPin);
    void logData(float message[], int length);
    void pullData();
  private:
    Sd2Card card;
    SdVolume volume;
    SdFile root;
};

#endif

