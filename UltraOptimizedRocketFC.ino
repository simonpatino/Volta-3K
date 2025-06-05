/*
  ULTRA-OPTIMIZED ROCKET FLIGHT COMPUTER
  For 3km IREC Competition - Performance First
  
  ZERO DELAYS - MAXIMUM EFFICIENCY
  
  Core optimizations:
  - Non-blocking operations only
  - Minimal memory allocation
  - Fast stage detection
  - Efficient LoRa transmission
  - Streamlined sensor reading
  - No complex math operations
*/

#include <LoRa.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>              // For data logging to external flash
#include "SparkFun_Ublox_Arduino_Library.h"


// BNO055 Operation Mode Constants (if not defined in library)
#ifndef OPERATION_MODE_NDOF
#define OPERATION_MODE_CONFIG                               (0X00)
#define OPERATION_MODE_ACCONLY                              (0X01)
#define OPERATION_MODE_MAGONLY                              (0X02)
#define OPERATION_MODE_GYRONLY                              (0X03)
#define OPERATION_MODE_ACCMAG                               (0X04)
#define OPERATION_MODE_ACCGYRO                              (0X05)
#define OPERATION_MODE_MAGGYRO                              (0X06)
#define OPERATION_MODE_AMG                                  (0X07)
#define OPERATION_MODE_IMUPLUS                              (0X08)
#define OPERATION_MODE_COMPASS                              (0X09)
#define OPERATION_MODE_M4G                                  (0X0A)
#define OPERATION_MODE_NDOF_FMC_OFF                         (0X0B)
#define OPERATION_MODE_NDOF                                 (0X0C)
#endif

// =============================================================================
// HARDWARE PINS (From Constants.h - Dual Storage System)
// =============================================================================
// LoRa pins
#define LORA_NSS 10
#define LORA_RST 9
#define LORA_DI0 29        // Updated from Constants.h
#define LORA_FREQ 918.250E6
#define LORA_SYNC 0xAF

// Storage pins - OPTIMIZED for external flash module
#define CS_FLASH 2              // External flash module (primary flight data)
#define CS_SD BUILTIN_SDCARD    // Built-in SD card (simulation data only)
#define FLASH_SPI_MOSI 11       // Flash SPI pins for high-speed data
#define FLASH_SPI_MISO 12
#define FLASH_SPI_SCK 13

// GPS pins
#define GPS_RX 0           // RX1 from Constants.h
#define GPS_TX 1           // TX1 from Constants.h

// LED pins (From Constants.h)
#define RLED 8             // Red LED
#define GLED 7             // Green LED  
#define BLED 6             // Blue LED
#define LED_STATUS RLED    // Use Red LED for status
#define LED_GPS GLED       // Use Green LED for GPS
#define LED_STAGE BLED     // Use Blue LED for stage

// Camera pin (From Constants.h)
#define CAMERA_PIN 23      // Real camera pin from Constants.h

// Verbose switch 
#define VERBOSE_SWITCH 20  // Pin 20 as specified by user

// Writer mode detection
bool writerMode = false;
bool normalMode = true;

// =============================================================================
// CONFIGURATION
// =============================================================================
bool VERBOSE = true;  // Will be dynamically set based on VERBOSE_SWITCH pin

// SMART ADAPTIVE TRANSMISSION SYSTEM
const unsigned long LORA_TX_INTERVAL_NORMAL = 500;   // 100ms = 10Hz (normal telemetry rate)
const unsigned long LORA_TX_INTERVAL_COMMAND_MODE = 500;  // 500ms = 2Hz (when commands detected)
const unsigned long COMMAND_ACTIVITY_TIMEOUT = 2000;     // Return to normal after 2 seconds of no commands

const unsigned long GPS_READ_INTERVAL = 1500;  // 1 second GPS updates
const unsigned long SENSOR_READ_INTERVAL = 25;  // 25ms = 40Hz sensor sampling (ultra fast!)
const unsigned long STATUS_TX_INTERVAL = 10000;  // 10 seconds for debugging status

// Smart transmission control variables
unsigned long lastCommandTime = 0;  // Track when last command was received
bool commandModeActive = false;     // Whether we're in slow transmission mode

// Flight detection thresholds
const float LAUNCH_ACCEL = 15.0;     // m/s² to detect launch
const float APOGEE_ACCEL = -3.0;     // m/s² to detect apogee
const float DROGUE_ALT = 1000.0;     // meters for drogue deploy
const float MAIN_ALT = 150.0;        // meters for main deploy

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
// Hardware objects
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);
Adafruit_BME280 bme;
SFE_UBLOX_GPS myGPS;

// Timing variables
unsigned long lastLoRaTime = 0;
unsigned long lastGpsTime = 0;
unsigned long lastSensorTime = 0;
unsigned long flightStartTime = 0;
unsigned long lastStatusTime = 0;  // For periodic status messages

// Communication variables - BULLETPROOF COMMAND SYSTEM (immediate processing)
bool groundConfirmation = false;

// Flight data - single struct for efficiency
struct {
    float iteration;
    float flightTime;
    float accelX, accelY, accelZ;
    float altitude;
    float pressure;
    float eulerX, eulerY, eulerZ;
    float gyroX, gyroY, gyroZ;  
    float humidity;             
    float maxAlt;
    float stage;
    float latitude;
    float longitude;
    float temperature;
    float gpsVelocity;  
    int satellites;
    bool gpsValid;
} flight;

// Flight stages
enum Stage {
    PAD = 1,
    BOOST = 2,
    COAST = 3,
    DROGUE = 4,
    MAIN = 5,
    LANDED = 6
} currentStage = PAD;

// System state
float baseAltitude = 0;
float refPressure = 1013.25;  // Will be calibrated during setup
unsigned long iteration = 0;
bool systemReady = false; 

// Ground station command compatibility
long pendingRocketFreqChangeHz = 0;

// Data IDs (matching ground station expectations)
const byte coreDataID = 0x00;       // Core telemetry data
const byte stringID = 0x02;         // String messages
const byte confirmationCode = 0x07;  // ACK confirmation

// Command IDs (matching ground station)
const byte CMD_GROUND_CONFIRM = 0x01;
const byte CMD_EJECTION_INFO = 0x05;
const byte CMD_CAMERA_ON = 0x06;
const byte CMD_CAMERA_OFF = 0x07;        // NOTE: Same as ACK - design issue in original
const byte CMD_SET_FREQUENCY = 0x08;
const byte ROCKET_FREQ_CHANGE_CONFIRM = 0x09;
const byte CMD_EXECUTE_FREQ_CHANGE = 0x0A;
const byte CMD_DELETE_VOLTA_FILE = 0x0B;

// Data transmission array (pre-allocated for speed)
float txData[12];

// Data logging support - OPTIMIZED (removed unused fullDataForLogging array)
File dataFile;
const char* dataFileName = "Volta.txt";
bool sdCardReady = false;

// =============================================================================
// SETUP - FAST INITIALIZATION
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(100); // Give serial time to stabilize
    
    // =============================================================================
    // 5-SECOND STARTUP DELAY WITH WRITER MODE DETECTION (Feature 1)
    // =============================================================================
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════════════╗");
    Serial.println("║    🚀 VOLTA-3K ULTRA-OPTIMIZED ROCKET FLIGHT COMPUTER 🚀     ║");
    Serial.println("║══════════════════════════════════════════════════════════════║");
    Serial.println("║  📝 WRITER MODE DETECTION: 10 SECOND WINDOW ACTIVE 📝         ║");
    Serial.println("║                                                              ║");
    Serial.println("║     ⚡ Type anything to enter WRITER MODE for file access     ║");
    Serial.println("║     🔥 Stay silent for NORMAL FLIGHT COMPUTER MODE           ║");
    Serial.println("╚══════════════════════════════════════════════════════════════╝");
    
    // 5-second startup timer with writer mode detection 
    float STAR_TIME = 10.0f;
    unsigned long startTime = millis();
    
    while ((STAR_TIME * 1000 - (millis() - startTime)) > 0) {
        if (Serial.available()) {
            writerMode = true;
            normalMode = false;
            
            // Visual indication of writer mode
            pinMode(RLED, OUTPUT);
            pinMode(BLED, OUTPUT);
            pinMode(GLED, OUTPUT);
            digitalWrite(RLED, HIGH);
            digitalWrite(BLED, HIGH);
            digitalWrite(GLED, HIGH);
            
       
            Serial.println();
            Serial.println("╔══════════════════════════════════════════════════════════════╗");
            Serial.println("║                    📝 WRITER MODE ACTIVE 📝                  ║");
            Serial.println("║══════════════════════════════════════════════════════════════║");
            Serial.println("║          🔧 File Access & Management System Ready            ║");
            Serial.println("║             ⏳ Initializing storage system...                ║");
            Serial.println("╚══════════════════════════════════════════════════════════════╝");
            
            delay(5000);
            break;
        }
    }
    
    // If writer mode detected, enter writer mode loop (from original main.ino)
    if (writerMode) {
        enterWriterMode();
        return; // Don't continue with normal setup
    }
    
    // Continue with normal rocket flight computer setup
    // Setup verbose switch pin
    pinMode(VERBOSE_SWITCH, INPUT);  // Pin 20 
    
    // =============================================================================
    // SMART STORAGE INITIALIZATION - FALLBACK SYSTEM (Feature 2)
    // =============================================================================
    if (VERBOSE) {
        Serial.println();
        Serial.println("╔══════════════════════════════════════════════════════════════╗");
        Serial.println("║                🔧 SYSTEM INITIALIZATION 🔧                   ║");
        Serial.println("╚══════════════════════════════════════════════════════════════╝");
        Serial.println("📦 Initializing smart storage system...");
    }
    
    Serial.print("💾 Checking external flash module...");
    
    // Try external flash first (CS_FLASH pin 2)
    pinMode(CS_FLASH, OUTPUT);
    digitalWrite(CS_FLASH, HIGH);  // Deselect flash initially
    
    if (SD.begin(CS_FLASH)) {
        Serial.println("✅ SUCCESS");
        if (VERBOSE) Serial.println("🎯 External flash module initialized successfully");
        sdCardReady = true;
        
        // Create optimized header for essential data only
        digitalWrite(CS_FLASH, LOW);  // Select flash for operations
        if (!SD.exists(dataFileName)) {
            dataFile = SD.open(dataFileName, FILE_WRITE);
            if (dataFile) {
                // OPTIMIZED HEADER - Only essential flight data (no useless KF/zeros)
                dataFile.println("Cycle,Time,Temp_C,Press_hPa,Alt_m,humidity,MaxAlt_m,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ,EulerX,EulerY,EulerZ,Stage,GPS_Lat,GPS_Lon,GPS_Valid,GPS_Sats");
                dataFile.close();
                if (VERBOSE) Serial.println("📄 Created optimized Volta.txt header");
            }
        } else {
            if (VERBOSE) Serial.println("📄 Volta.txt exists on external flash");
        }
        digitalWrite(CS_FLASH, HIGH);  // Deselect flash
    } 
    // Fallback to built-in SD card if external flash fails
    else if (SD.begin(BUILTIN_SDCARD)) {
        Serial.println("⚠️  External flash failed, using built-in SD");
        if (VERBOSE) Serial.println("🔄 Fallback: Built-in SD card initialized");
        sdCardReady = true;
        
        if (!SD.exists(dataFileName)) {
            dataFile = SD.open(dataFileName, FILE_WRITE);
            if (dataFile) {
                // Same optimized header for built-in SD
                dataFile.println("Cycle,Time,Temp_C,Press_hPa,Alt_m,humidity,MaxAlt_m,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ,EulerX,EulerY,EulerZ,Stage,GPS_Lat,GPS_Lon,GPS_Valid,GPS_Sats");
                dataFile.close();
                if (VERBOSE) Serial.println("📄 Created optimized Volta.txt header on built-in SD");
            }
        } else {
            if (VERBOSE) Serial.println("📄 Volta.txt exists on built-in SD");
        }
    } 
    else {
        Serial.println("❌ Both storage systems failed! Continuing without data logging.");
        sdCardReady = false;
    }
    
    // =============================================================================
    // 🔧 HARDWARE INITIALIZATION - ENHANCED VERBOSE FEEDBACK
    // =============================================================================
    if (VERBOSE) {
        Serial.println("🔧 Starting hardware initialization sequence...");
        Serial.println();
    }
    
    // LED setup with enhanced feedback
    if (VERBOSE) Serial.print("💡 Initializing LED system...");
    pinMode(LED_STATUS, OUTPUT);
    pinMode(LED_GPS, OUTPUT);
    pinMode(LED_STAGE, OUTPUT);
    if (VERBOSE) Serial.println("✅ LED pins configured");
    
    // Camera pin setup with visual feedback
    if (VERBOSE) Serial.print("📹 Initializing camera system...");
    pinMode(CAMERA_PIN, OUTPUT);
    digitalWrite(CAMERA_PIN, LOW);  // Start with camera off
    if (VERBOSE) Serial.println("✅ Camera pin configured (OFF)");
    
    // Enhanced startup LED sequence with description
    if (VERBOSE) Serial.print("🌟 Performing LED startup sequence...");
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
        digitalWrite(LED_STATUS, LOW);
        delay(100);
    }
    if (VERBOSE) Serial.println("✅ LED test complete");
    
    // Check verbose switch with better feedback
    pinMode(VERBOSE_SWITCH, INPUT);
    VERBOSE = digitalRead(VERBOSE_SWITCH) == HIGH;
    if (VERBOSE) {
        Serial.println();
        Serial.println("╔══════════════════════════════════════════════════════════════╗");
        Serial.println("║              🚀 ULTRA-OPTIMIZED ROCKET FC v3.0               ║");
        Serial.println("║                   Fast Hardware Initialization               ║");
        Serial.println("╚══════════════════════════════════════════════════════════════╝");
    }
    
    // Enhanced LoRa initialization with detailed feedback
    if (VERBOSE) {
        Serial.println("📡 Initializing LoRa communication system...");
        Serial.println("   ⚙️  Setting up pins and frequency configuration");
    }
    
    LoRa.setPins(LORA_NSS, LORA_RST, LORA_DI0);
    if (!LoRa.begin(LORA_FREQ)) {
        if (VERBOSE) Serial.println("❌ LoRa initialization FAILED - entering error loop");
        errorLoop();
    }
    
    LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(17);  // +17dBm for 3km range
    LoRa.setSpreadingFactor(10);
    LoRa.setSignalBandwidth(250E3);
    LoRa.onReceive(onReceive);
    
    // CRITICAL: Start listening for commands immediately after setup
    LoRa.receive();
    
    if (VERBOSE) {
        Serial.println("✅ LoRa system ready!");
        Serial.println("   📶 Frequency: 918.250 MHz");
        Serial.println("   🔊 TX Power: +17dBm (3km range)");
        Serial.println("   🎯 Sync Word: 0xAF");
        Serial.println("   📻 Command reception active");
    }
    
    // Enhanced sensor initialization with visual feedback
    if (VERBOSE) {
        Serial.println();
        Serial.println("🔬 Initializing precision sensor suite...");
    }
    
    // Initialize sensors with correct I2C setup
    if (VERBOSE) Serial.print("🔌 Starting I2C bus (Wire2)...");
    Wire2.begin();  // Initialize Wire2 for sensors
    if (VERBOSE) Serial.println("✅ I2C ready");
    
    // BNO055 9-DOF IMU initialization
    if (VERBOSE) Serial.print("🧭 Connecting to BNO055 9-DOF IMU...");
    if (!bno.begin()) {
        if (VERBOSE) Serial.println("❌ BNO055 FAILED - entering error loop");
        errorLoop();
    }
    if (VERBOSE) Serial.println("✅ BNO055 connected");
    
    // BME280 Environmental sensor initialization
    if (VERBOSE) Serial.print("🌡️  Connecting to BME280 environmental sensor...");
    if (!bme.begin(0x77, &Wire2)) {  // BME280 on address 0x77 using Wire2
        if (VERBOSE) Serial.println("❌ BME280 FAILED - entering error loop");
        errorLoop();
    }
    if (VERBOSE) Serial.println("✅ BME280 connected");
    
    if (VERBOSE) Serial.println("✅ Optimized for flight");
    
    // Configure BNO055 for maximum precision
    if (VERBOSE) Serial.print("🎯 Configuring BNO055 for high precision...");
    bno.setExtCrystalUse(true);
    // Set BNO055 to NDOF mode (fixed compilation issue)
    bno.setMode(OPERATION_MODE_NDOF);
    if (VERBOSE) Serial.println("✅ NDOF mode active");
    
    // GPS initialization with enhanced feedback
    if (VERBOSE) Serial.print("🛰️  Connecting to u-blox GPS module...");
    if (!myGPS.begin(Wire2)) {
        if (VERBOSE) Serial.println("❌ GPS FAILED - entering error loop");
        errorLoop();
    }
    if (VERBOSE) Serial.println("✅ GPS connected");
    
    if (VERBOSE) Serial.print("📡 Configuring GPS for optimal performance...");
    myGPS.setI2COutput(COM_TYPE_UBX);  // Set I²C to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration();          // Save settings to flash and BBR
    if (VERBOSE) Serial.println("✅ GPS optimized");
    
    // Enhanced calibration sequence with progress feedback
    if (VERBOSE) {
        Serial.println();
        Serial.println("🎯 Starting precision calibration sequence...");
    }
    
    // Calculate reference pressure (like original setReferencePressure)
    float tempRefPressure = 0;
    if (VERBOSE) Serial.print("📏 Calibrating barometric pressure baseline (20 samples)...");
    for (int i = 0; i < 20; i++) {
        tempRefPressure += bme.readPressure() / 100.0F;  // hPa
        delay(50);  // Short delay for averaging
        if (VERBOSE && i % 5 == 0) Serial.print(".");  // Progress dots
    }
    refPressure = tempRefPressure / 20.0;
    if (VERBOSE) {
        Serial.println("✅");
        Serial.print("   🌊 Reference pressure: ");
        Serial.print(refPressure, 2);
        Serial.println(" hPa");
    }
    
    // Quick altitude calibration using reference pressure
    float altSum = 0;
    if (VERBOSE) Serial.print("🏔️  Calibrating altitude baseline (10 samples)...");
    for (int i = 0; i < 10; i++) {
        altSum += bme.readAltitude(refPressure);
        delay(50);  // Only delay during calibration
        if (VERBOSE && i % 3 == 0) Serial.print(".");  // Progress dots
    }
    baseAltitude = altSum / 10.0;
    if (VERBOSE) {
        Serial.println("✅");
        Serial.print("   🎯 Launch pad altitude: ");
        Serial.print(baseAltitude, 1);
        Serial.println(" m");
    }
    
    // Enhanced system finalization
    if (VERBOSE) {
        Serial.println();
        Serial.println("🚀 Finalizing flight computer initialization...");
    }
    
    // Initialize flight data
    memset(&flight, 0, sizeof(flight));
    flight.altitude = 0;
    flight.maxAlt = 0;
    
    flightStartTime = millis();
    systemReady = true;
    digitalWrite(LED_STATUS, HIGH);
    
    if (VERBOSE) {
        Serial.println("✅ Flight data structure initialized");
        Serial.println("✅ System ready indicator ON");
        Serial.println();
        Serial.println("╔══════════════════════════════════════════════════════════════╗");
        Serial.println("║              🎉 INITIALIZATION COMPLETE! 🎉                  ║");
        Serial.println("║══════════════════════════════════════════════════════════════║");
        Serial.println("║  🚀 Ultra-Optimized Rocket Flight Computer READY             ║");
        Serial.println("║  📡 LoRa communication: ACTIVE                               ║");
        Serial.println("║  🔬 Sensor suite: CALIBRATED                                 ║");
        Serial.println("║  💾 Data logging: READY                                      ║");
        Serial.println("║  🎯 Flight detection: ARMED                                  ║");
        Serial.println("║                                                              ║");
        Serial.println("║               🔥 READY FOR LAUNCH! 🔥                        ║");
        Serial.println("╚══════════════════════════════════════════════════════════════╝");
        Serial.println();
    }
}

// =============================================================================
// MAIN LOOP - ZERO BLOCKING DELAYS
// =============================================================================
void loop() {
    // Only run main loop in normal mode (not writer mode)
    if (!normalMode) {
        return;
    }
    
    unsigned long now = millis();
    
    // Update verbose setting dynamically based on switch
    VERBOSE = digitalRead(VERBOSE_SWITCH) == HIGH;
    
    // SMART ADAPTIVE TRANSMISSION CONTROL
    // Check if we should switch transmission modes based on command activity
    if (commandModeActive && (now - lastCommandTime > COMMAND_ACTIVITY_TIMEOUT)) {
        commandModeActive = false;  // Return to normal fast rate
        if (VERBOSE) Serial.println("🚀📡 Returning to fast telemetry mode - command processing complete");
    }
    
    // Determine current transmission interval based on mode
    unsigned long currentTxInterval = commandModeActive ? LORA_TX_INTERVAL_COMMAND_MODE : LORA_TX_INTERVAL_NORMAL;
    
    // Sensor reading (highest priority)
    if (now - lastSensorTime >= SENSOR_READ_INTERVAL) {
        readSensors();
        updateStage();
        
        // =============================================================================
        // COMPREHENSIVE DATA LOGGING - Log all sensor data 
        // =============================================================================
        logComprehensiveData();
        
        lastSensorTime = now;
    }
    
    // GPS reading (lower priority)
    if (now - lastGpsTime >= GPS_READ_INTERVAL) {
        readGPS();
        lastGpsTime = now;
    }
    
    // SMART LoRa transmission - adapts speed based on command activity
    if (now - lastLoRaTime >= currentTxInterval) {
        transmitData();
        iteration++;
        lastLoRaTime = now;
    }
    
    // Handle commands - IMMEDIATE PROCESSING IN INTERRUPT (like hardware test)
    // Ensure LoRa stays in receive mode
    static unsigned long lastReceiveCheck = 0;
    if (millis() - lastReceiveCheck > 100) {  // Check every 100ms
        LoRa.receive();
        lastReceiveCheck = millis();
    }
    
    // Periodic status transmission for debugging (every 10 seconds)
    if (VERBOSE && now - lastStatusTime >= STATUS_TX_INTERVAL) {
        String statusMsg = "🚀 ROCKET ALIVE - Mode: ";
        statusMsg += commandModeActive ? "COMMAND(2Hz)" : "NORMAL(10Hz)";
        statusMsg += " 🛰️ GPS: ";
        statusMsg += flight.gpsValid ? "VALID" : "INVALID";
        statusMsg += " Sats: ";
        statusMsg += flight.satellites;
        transmitString(statusMsg);
        
        // Enhanced GPS debug info with visual indicators
        Serial.println();
        Serial.println("📊 SYSTEM STATUS REPORT:");
        Serial.print("   🛰️  GPS Status: ");
        if (flight.gpsValid && flight.satellites >= 4) {
            Serial.println("✅ EXCELLENT LOCK");
        } else if (flight.gpsValid) {
            Serial.println("🟡 BASIC LOCK");
        } else if (flight.satellites > 0) {
            Serial.println("🟠 SEARCHING");
        } else {
            Serial.println("❌ NO SIGNAL");
        }
        Serial.print("   📡 Satellites: ");
        Serial.println(flight.satellites);
        if (flight.gpsValid) {
            Serial.print("   🌍 Position: ");
            Serial.print(flight.latitude, 6);
            Serial.print(", ");
            Serial.println(flight.longitude, 6);
        }
        Serial.print("   📶 Transmission Mode: ");
        Serial.println(commandModeActive ? "🐌 COMMAND " : "🚀 NORMAL ");
        
        lastStatusTime = now;
    }
    
    // Update LEDs (non-blocking)
    updateStatus();
}

// =============================================================================
// SENSOR READING - OPTIMIZED
// =============================================================================
void readSensors() {
    // Read acceleration (most important for stage detection)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    flight.accelX = accel.x();
    flight.accelY = accel.y();
    flight.accelZ = accel.z();
    
    // Read orientation
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    flight.eulerX = euler.x();
    flight.eulerY = euler.y();
    flight.eulerZ = euler.z();
      
    // Read gyroscope data (angular velocity)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    flight.gyroX = gyro.x();
    flight.gyroY = gyro.y();
    flight.gyroZ = gyro.z();
    
    // Read barometer
    flight.pressure = bme.readPressure() / 100.0;
    flight.altitude = bme.readAltitude(refPressure) - baseAltitude;
    flight.temperature = bme.readTemperature();
    flight.humidity = bme.readHumidity();  // Read humidity data

    
    // Update max altitude
    if (flight.altitude > flight.maxAlt) {
        flight.maxAlt = flight.altitude;
    }
    
    // Update timing
    flight.flightTime = (millis() - flightStartTime) / 1000.0;
    flight.iteration = iteration;
    flight.stage = currentStage;
}

// =============================================================================
// GPS READING - NON-BLOCKING (SparkFun u-blox GPS)
// =============================================================================
void readGPS() {
    // Check for new data from GPS module
    myGPS.checkUblox();
    
    // Get satellite count
    int satellites = myGPS.getSIV();
    
    // Check if we have a good fix (3+ satellites minimum)
    if (satellites >= 3) {
        // Convert from integer degrees (1e-7) to float
        float newLat = myGPS.getLatitude() / 10000000.0;
        float newLon = myGPS.getLongitude() / 10000000.0;
        
        // Calculate velocity from NED components (mm/s to m/s)
        float v_n = myGPS.getNedNorthVel() / 1000.0;
        float v_e = myGPS.getNedEastVel() / 1000.0;
        float v_d = myGPS.getNedDownVel() / 1000.0;
        float gpsVelocity = sqrt(v_n*v_n + v_e*v_e + v_d*v_d);
        
        // Update flight data
        flight.latitude = newLat;
        flight.longitude = newLon;
        flight.satellites = satellites;
        flight.gpsVelocity = gpsVelocity;
        flight.gpsValid = true;
        
        
    } else {
        // No valid fix
        flight.gpsValid = false;
        flight.satellites = satellites;  // Still update satellite count for debugging
               
    }
}

// =============================================================================
// STAGE DETECTION - FAST AND SIMPLE WITH ENHANCED VERBOSE FEEDBACK
// =============================================================================
void updateStage() {
    float totalAccel = sqrt(flight.accelX * flight.accelX + 
                           flight.accelY * flight.accelY + 
                           flight.accelZ * flight.accelZ);
    
    switch (currentStage) {
        case PAD:
            if (totalAccel > LAUNCH_ACCEL & groundConfirmation) {
                currentStage = BOOST;
                if (VERBOSE) {
                    Serial.println();
                    Serial.println("🚀🔥 LAUNCH DETECTED! 🔥🚀");
                    Serial.print("   💨 Total acceleration: ");
                    Serial.print(totalAccel, 1);
                    Serial.println(" m/s²");
                    Serial.println("   🎯 Stage: PAD → BOOST");
                }
            }
            break;
            
        case BOOST:
            if (totalAccel < APOGEE_ACCEL && flight.altitude > 100) {
                currentStage = COAST;
                if (VERBOSE) {
                    Serial.println();
                    Serial.println("🌌 COASTING PHASE INITIATED 🌌");
                    Serial.print("   📈 Current altitude: ");
                    Serial.print(flight.altitude, 1);
                    Serial.println(" m");
                    Serial.println("   🎯 Stage: BOOST → COAST");
                }
            }
            break;
            
        case COAST:
            if (flight.accelZ < -8.0 && flight.altitude > DROGUE_ALT) {
                currentStage = DROGUE;
                if (VERBOSE) {
                    Serial.println();
                    Serial.println("🪂 DROGUE DEPLOYMENT ALTITUDE 🪂");
                    Serial.print("   📉 Altitude: ");
                    Serial.print(flight.altitude, 1);
                    Serial.println(" m");
                    Serial.println("   ⚠️  Note: No pyrotechnics in this build");
                    Serial.println("   🎯 Stage: COAST → DROGUE");
                }
            }
            break;
            
        case DROGUE:
            if (flight.altitude < MAIN_ALT) {
                currentStage = MAIN;
                if (VERBOSE) {
                    Serial.println();
                    Serial.println("🎯 MAIN CHUTE ALTITUDE REACHED 🎯");
                    Serial.print("   📉 Altitude: ");
                    Serial.print(flight.altitude, 1);
                    Serial.println(" m");
                    Serial.println("   ⚠️  Note: No pyrotechnics in this build");
                    Serial.println("   🎯 Stage: DROGUE → MAIN");
                }
            }
            break;
            
        case MAIN:
            if (flight.altitude < 20 && totalAccel < 12) {
                currentStage = LANDED;
                if (VERBOSE) {
                    Serial.println();
                    Serial.println("🎊🎉 TOUCHDOWN! MISSION COMPLETE! 🎉🎊");
                    Serial.print("   🏁 Final altitude: ");
                    Serial.print(flight.altitude, 1);
                    Serial.println(" m");
                    Serial.print("   💫 Maximum altitude achieved: ");
                    Serial.print(flight.maxAlt, 1);
                    Serial.println(" m");
                    Serial.println("   🎯 Stage: MAIN → LANDED");
                    Serial.println("   🏆 Flight computer mission SUCCESS!");
                }
            }
            break;
    }
}

// =============================================================================
// DATA TRANSMISSION - OPTIMIZED FOR COMMAND RECEPTION
// =============================================================================
void transmitData() {
    // Quick check: Skip transmission if we're currently receiving a command
    static unsigned long lastTxTime = 0;
    unsigned long now = millis();
    
    // Pack data efficiently - pre-built string for speed
    static char dataBuffer[200];
    sprintf(dataBuffer, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.5f,%.5f",
            flight.iteration,
            flight.accelX, flight.accelY, flight.accelZ,
            flight.eulerX, flight.eulerY, flight.eulerZ,
            flight.altitude,
            (float)currentStage,
            flight.gpsVelocity,
            flight.latitude, flight.longitude);
    
    // FAST transmission with minimal blocking time
    LoRa.beginPacket();
    LoRa.write(coreDataID);  // 0x00 - Core data packet ID expected by GS
    LoRa.print(dataBuffer);
    LoRa.endPacket();
    
    // CRITICAL: Return to receive mode IMMEDIATELY
    LoRa.receive();
    
    lastTxTime = now;
    
  
}

// =============================================================================
// BRAND NEW EFFICIENT LORA COMMAND SYSTEM
// =============================================================================

// LoRa Reception - Interrupt-based for maximum efficiency
void onReceive(int packetSize) {
    if (packetSize > 0) {
        // SMART TRANSMISSION: Trigger slow mode when command received
        lastCommandTime = millis();
        if (!commandModeActive) {
            commandModeActive = true;
            if (VERBOSE) Serial.println("\n🐌📡 Switching to slow telemetry mode for reliable commands");
        }
        
        // Read the command byte (always first byte)
        byte command = LoRa.read();
        
        // Read any additional payload (for frequency change)
        String payload = "";
        while (LoRa.available()) {
            payload += (char)LoRa.read();
        }
        
        if (VERBOSE) {
            Serial.print("\n📨🎯 COMMAND RECEIVED: 0x");
            Serial.print(command, HEX);
            if (payload.length() > 0) {
                Serial.print(" | PAYLOAD: ");
                Serial.print(payload);
            }
            Serial.print(" | RSSI: ");
            Serial.print(LoRa.rssi());
            Serial.println(" dBm");
        }
        
        // Send immediate ACK (0x07) for all commands
        LoRa.beginPacket();
        LoRa.write(0x07);  // ACK
        LoRa.endPacket();
        
        // Process commands IMMEDIATELY like the working hardware test
        switch (command) {
            case 0x01:  // Ground Confirmation
                groundConfirmation = true;
                if (VERBOSE) Serial.println("   ✅🎯 Ground station confirmed flight readiness!");
                break;
                
            case 0x06:  // Camera ON
                if (VERBOSE) Serial.println("   📹🟢 Camera system ACTIVATED");
                digitalWrite(CAMERA_PIN, HIGH);
                break;
                
            case 0x07:  // Camera OFF
                if (VERBOSE) Serial.println("   📹🔴 Camera system DEACTIVATED");
                digitalWrite(CAMERA_PIN, LOW);
                break;
                
            case 0x08:  // Frequency Change Request
                if (payload.length() > 0) {
                    long newFrequency = atol(payload.c_str());
                    if (newFrequency >= 902E6 && newFrequency <= 928E6) {
                        if (VERBOSE) {
                            Serial.print("   📡🔄 Frequency change request: ");
                            Serial.print(newFrequency / 1E6, 1);
                            Serial.println(" MHz - VALID");
                        }
                        // Send confirmation
                        LoRa.beginPacket();
                        LoRa.write(0x09);
                        LoRa.print(payload);
                        LoRa.endPacket();
                        pendingRocketFreqChangeHz = newFrequency;
                        if (VERBOSE) Serial.println("   ✅ Confirmation sent to ground station");
                    } else {
                        if (VERBOSE) Serial.println("   ❌ Invalid frequency range - request rejected");
                    }
                }
                break;
                
            case 0x0A:  // Execute Frequency Change
                if (pendingRocketFreqChangeHz != 0) {
                    if (VERBOSE) {
                        Serial.print("   📡⚡ Executing frequency change to: ");
                        Serial.print(pendingRocketFreqChangeHz / 1E6, 1);
                        Serial.println(" MHz");
                    }
                    LoRa.setFrequency(pendingRocketFreqChangeHz);
                    pendingRocketFreqChangeHz = 0;
                    if (VERBOSE) Serial.println("   ✅ Frequency change completed successfully!");
                } else {
                    if (VERBOSE) Serial.println("   ⚠️  No pending frequency change to execute");
                }
                break;
                
            case 0x0B:  // Delete File
                if (VERBOSE) Serial.println("   🗑️📄 File deletion command received");
                // =============================================================================
                // DELETE VOLTA.txt COMMAND IMPLEMENTATION (Feature 3)
                // =============================================================================
                deleteVoltaFile();
                break;
                
            default:
                if (VERBOSE) {
                    Serial.print("   ❌🤔 Unknown command: 0x");
                    Serial.println(command, HEX);
                }
                break;
        }
    }
}

void sendAck() {
    LoRa.beginPacket();
    LoRa.write(confirmationCode);  // 0x07 - ACK
    LoRa.endPacket();
    LoRa.receive();
    
    if (VERBOSE) {
        Serial.println("   📤✅ ACK transmitted to ground station (0x07)");
    }
}

void checkReceive() {
    // Exact copy of original LoRaComm::checkReceive behavior
    // Comment from original: "Si esta funcion no insiste en el .receive() el callback es mucho menos efectivo"
    LoRa.receive();
    
    if (VERBOSE) {
        static unsigned long lastReceiveDebug = 0;
        if (millis() - lastReceiveDebug > 10000) {  // Every 10 seconds
            Serial.println("📻🎧 LoRa actively listening for ground station commands...");
            lastReceiveDebug = millis();
        }
    }
}

void transmitString(String message) {
    // Transmit string messages with proper packet ID
    LoRa.beginPacket();
    LoRa.write(stringID);  // 0x02 - String message packet ID
    LoRa.print(message);
    LoRa.endPacket();
    LoRa.receive();  // Back to receive mode immediately
}

// =============================================================================
// STATUS UPDATES - CLEAN LED SYSTEM
// =============================================================================
void updateStatus() {
    // RED LED (STATUS): System ready and health
    digitalWrite(LED_STATUS, systemReady);
    
    // GREEN LED (GPS): GPS status - simple and clear
    static unsigned long gpsLedTime = 0;
    if (millis() - gpsLedTime > 1000) {  // Update every second
        if (flight.gpsValid && flight.satellites >= 4) {
            digitalWrite(LED_GPS, HIGH);  // Solid green = GPS lock with 4+ satellites
        } else if (flight.satellites > 0) {
            // Blink green = GPS searching (has satellites but no fix)
            digitalWrite(LED_GPS, !digitalRead(LED_GPS));
        } else {
            digitalWrite(LED_GPS, LOW);   // Off = No GPS signal at all
        }
        gpsLedTime = millis();
    }
    
    // BLUE LED (STAGE): Flight stage indication
    static unsigned long stageLedTime = 0;
    static bool stageLedState = false;
    
    unsigned long interval = 1000;
    switch (currentStage) {
        case PAD:     interval = 2000; break;  // Slow blink - waiting on pad
        case BOOST:   interval = 100;  break;  // Fast blink - boosting
        case COAST:   interval = 300;  break;  // Medium blink - coasting
        case DROGUE:  interval = 500;  break;  // Slow-medium blink - drogue
        case MAIN:    interval = 200;  break;  // Fast-medium blink - main
        case LANDED:  
            digitalWrite(LED_STAGE, HIGH);     // Solid on - landed
            return;  // No blinking when landed
    }
    
    if (millis() - stageLedTime > interval) {
        stageLedState = !stageLedState;
        digitalWrite(LED_STAGE, stageLedState);
        stageLedTime = millis();
    }
}

// =============================================================================
// ERROR HANDLING
// =============================================================================
void errorLoop() {
    while (1) {
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
        digitalWrite(LED_STATUS, LOW);
        delay(100);
    }
}

// =============================================================================
// WRITER MODE IMPLEMENTATION (Feature 1 Support)
// =============================================================================
void enterWriterMode() {
    // Initialize storage system before writer mode operations
    Serial.print("Initializing storage for writer mode...");
    
    // Try external flash first (CS_FLASH pin 2)
    pinMode(CS_FLASH, OUTPUT);
    digitalWrite(CS_FLASH, HIGH);  // Deselect flash initially
    
    if (SD.begin(CS_FLASH)) {
        Serial.println("External flash module initialized successfully");
        sdCardReady = true;
    } 
    // Fallback to built-in SD card if external flash fails
    else if (SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Fallback: Using built-in SD card");
        sdCardReady = true;
    } 
    else {
        Serial.println("FAILED - No storage available");
        sdCardReady = false;
        // Still allow writer mode to run, but file operations will show error messages
    }

    Serial.println("***************************");
    Serial.println("* Write 'R' for Volta.txt *");
    Serial.println("* Write 'P' for pyro.txt *");
    Serial.println("***************************");
    
    // Clear serial buffer
    while (Serial.available() > 0) {
        Serial.read();
    }
    
    // Wait for command
    while (Serial.available() == 0) {
        ; // Wait for input
    }
    
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'R') {
            sendFileContent("Volta.txt");
        } else if (command == 'P') {
            sendFileContent("pyro.txt");
        }
    }
    
    // Stay in writer mode forever (like original)
    while (true) {
        ; // Do nothing forever
    }
}

void sendFileContent(const char* filename) {
    if (!sdCardReady) {
        Serial.println("Storage not available");
        return;
    }
    
    Serial.print("Reading file: ");
    Serial.println(filename);
    
    // Smart storage: SD.begin() already configured the right storage (external flash or built-in SD)
    File myFile = SD.open(filename);
    if (myFile) {
        Serial.print("File size: ");
        Serial.print(myFile.size());
        Serial.println(" bytes");
        Serial.println("--- FILE CONTENT START ---");
        
        while (myFile.available()) {
            Serial.write(myFile.read());
        }
        
        Serial.println();
        Serial.println("--- FILE CONTENT END ---");
        myFile.close();
        
        Serial.print("Successfully sent: ");
        Serial.println(filename);
    } else {
        Serial.print("Error: Could not open ");
        Serial.println(filename);
        Serial.println("Make sure the file exists on the storage device.");
    }
}

// =============================================================================
// DELETE FILE COMMAND IMPLEMENTATION (Feature 3)
// =============================================================================
void deleteVoltaFile() {
    if (!sdCardReady) {
        if (VERBOSE) Serial.println("Storage not available for file deletion");
        return;
    }
    
    // Smart storage: SD.begin() already configured the right storage (external flash or built-in SD)
    if (SD.exists(dataFileName)) {
        if (SD.remove(dataFileName)) {
            if (VERBOSE) {
                Serial.print("File deleted successfully: ");
                Serial.println(dataFileName);
            }
        } else {
            if (VERBOSE) {
                Serial.print("Failed to delete file: ");
                Serial.println(dataFileName);
            }
        }
    } else {
        if (VERBOSE) {
            Serial.print("File does not exist: ");
            Serial.println(dataFileName);
        }
    }
}

// =============================================================================
// OPTIMIZED DATA LOGGING - ESSENTIAL FLIGHT DATA ONLY (Feature 2)
// =============================================================================
void logComprehensiveData() {
    if (!sdCardReady) {
        return; // Skip logging if storage not available
    }
    
    // SMART STORAGE: Only log meaningful data, skip zeros and useless KF data
    // Open file for writing (works with both external flash and built-in SD)
    dataFile = SD.open(dataFileName, FILE_WRITE);
    if (dataFile) {
        // OPTIMIZED DATA: Only essential flight data (17 fields vs 33 original)
        // Format: Cycle,Time,Temp_C,Press_hPa,Alt_m,humidity,MaxAlt_m,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ,EulerX,EulerY,EulerZ,Stage,GPS_Lat,GPS_Lon,GPS_Valid,GPS_Sats
        
        dataFile.print(flight.iteration, 0);        // Cycle (no decimals)
        dataFile.print(",");
        dataFile.print(flight.flightTime, 3);       // Time (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.temperature, 2);      // Temperature (2 decimals)
        dataFile.print(",");
        dataFile.print(flight.pressure, 2);         // Pressure (2 decimals)
        dataFile.print(",");
        dataFile.print(flight.altitude, 3);         // Altitude (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.humidity, 3);
        dataFile.print(",");
        dataFile.print(flight.maxAlt, 3);           // Max altitude (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.accelX, 3);           // Accel X (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.accelY, 3);           // Accel Y (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.accelZ, 3);           // Accel Z (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.gyroX, 3);           // Accel X (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.gyroY, 3);           // Accel Y (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.gyroZ, 3);           // Accel Z (3 decimals)
        dataFile.print(",");
        dataFile.print(flight.eulerX, 2);           // Euler X (2 decimals)
        dataFile.print(",");
        dataFile.print(flight.eulerY, 2);           // Euler Y (2 decimals)
        dataFile.print(",");
        dataFile.print(flight.eulerZ, 2);           // Euler Z (2 decimals)
        dataFile.print(",");
        dataFile.print((int)currentStage);          // Stage (integer)
        dataFile.print(",");
        
        // GPS data - only if valid, otherwise skip to save space
        if (flight.gpsValid) {
            dataFile.print(flight.latitude, 6);     // GPS Lat (6 decimals for precision)
            dataFile.print(",");
            dataFile.print(flight.longitude, 6);    // GPS Lon (6 decimals for precision)
            dataFile.print(",");
            dataFile.print("1");                    // GPS Valid = 1
        } else {
            dataFile.print("0,0,0");                // No GPS data
        }
        dataFile.print(",");
        dataFile.print(flight.satellites);          // Satellite count
        
        dataFile.println(); // End line
        dataFile.close();
        
        // Performance optimization: Only print debug every 10 seconds
        static unsigned long lastLogDebug = 0;
        if (VERBOSE && millis() - lastLogDebug > 10000) {
            Serial.println("\n💾📝 Flight data successfully logged to storage");
            Serial.print("   📈 Total iterations: ");
            Serial.println(flight.iteration);
            Serial.print("   🎯 Current stage: ");
            switch(currentStage) {
                case PAD: Serial.println("🏁 PAD"); break;
                case BOOST: Serial.println("🚀 BOOST"); break;
                case COAST: Serial.println("🌌 COAST"); break;
                case DROGUE: Serial.println("🪂 DROGUE"); break;
                case MAIN: Serial.println("🎯 MAIN"); break;
                case LANDED: Serial.println("🎊 LANDED"); break;
            }
            lastLogDebug = millis();
        }
    }
}
