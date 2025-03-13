#ifndef MEMORYMANAGER_H
#define MEMORYMANAGER_H
#include <SD.h>
#include <map>

class MemoryManager {
  public:
    File dataFile;
    File simFile; //File used to pull data for SITL
    MemoryManager();
    bool begin(int csPin);
    void logFloatData(float message[], int length, const char* fileName, bool withGPS = false);
    void logBoolData(bool message[], float time, int length, const char* fileName);
    bool readSimulatedData(std::map<String, float> &dataDict);
    void deleteFiles();
    int startSimulationData();
    char kfRealFileName[12] = "kf_real.csv"; //Real data from the simulation
    char kfMeasuFileName[14] = "kf_measu.csv"; //Measurement data sensed 
    char kfConfigFileName[14] = "kf_config.csv"; //time and x_dim for automatic plotting 
    char kfOutputFileName[14] = "kf_output.csv"; //Kalman Filter output state
    char kfPerformanceFileName[19] = "kf_performance.csv"; //Kalman Filter output state
    char dataFileName[10] = "Volta.txt";
    char pyroFileName[9] = "pyro.txt";
  private:
    char simFileName[19] = "simDataOpenSim.csv";
    int csPin;
    //char simFileName[19] = "simDataDragged.csv";
    //char simFileName[18] = "simDataLinear.csv";
    //char simFileName[12] = "simData.csv";
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    int numColumns = 1; //Needs to start on 1
    void checkAndDeleteFile(const char* filename);
};

#endif

