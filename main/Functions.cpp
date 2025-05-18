#include "Functions.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <LoRa.h> // Ensure LoRa library is included for direct use
#include "Constants.h" // Include Constants to get CMD_EXECUTE_FREQ_CHANGE

// Define the new confirmation ID
#define ROCKET_FREQ_CHANGE_CONFIRM 0x09

// Define new command ID for deleting Volta.txt
#define CMD_DELETE_VOLTA_FILE 0x0B
// Define ACK command ID that rocket sends (matches GS expectation)
#define CMD_ACK_FROM_ROCKET 0x07


// Global variable to store pending frequency change requested by GS
long pendingRocketFreqChangeHz = 0;

// ====================== Global Function Implementations ======================

void serialPrintMessage() {
  Serial.print("Message: ");
  for (int i = 0; i < messCoreLenght; i++) {
    Serial.print(messageCore[i], 2);
    Serial.print(", ");
  }
   Serial.print("\n");
  // for (int i = 0; i < 10; i++) {
  //   Serial.print(continuityPyros[i]);
  //   Serial.print(", ");
  // }
  // Serial.print("\t");
  // /*   for (int i = 0; i < 4; i++) {
  //   Serial.print(powderChambTemp[i], 0);
  //   Serial.print(", ");
  // } */
  // for (int i = 0; i < x_dim; i++) {
  //   Serial.print(kalmanState(i), 1);
  //   Serial.print(", ");
  // }
  // Serial.println("");
}

/*
  * Samples data from the rocket enviroment or it pulls data from the SD card if it is a Simulation
*/
void sample() {
  static float prevSampleTime = 0.0f;
  gps.updateGPS(false);
  //pyro.checkContinuityAll(continuityPyros);

  if (IS_SIMULATION) {
    currentData["intTime"] = 0;
    while (currentData["intTime"] < (sampleDelay / 1000.)) {  //This while loop is so we can manage slowed-reading rates for KF performance and still have a high resolution data file
      if (mem.readSimulatedData(currentData)) {
        //Les agregamos ruido gausiano de media 0 a las dos mediciones que podemos tomar
        currentData["alt"] = addGaussianNoise(currentData["realAlt"], 0, baroStdDev);
        currentData["linAccData1"] = addGaussianNoise(currentData["realAccData1"], 0, accelStdDev);
        currentData["accData1"] = addGaussianNoise(currentData["realAccData1"], 0, accelStdDev);
        currentData["intTime"] = currentData["time"] - prevSampleTime;  //Interval between to consecutive readings

        float realData[x_dim + 1];
        realData[0] = currentData["time"];
        realData[1] = currentData["realAlt"];
        realData[2] = currentData["rawVel"];
        realData[3] = currentData["realAccData1"];
        mem.logFloatData(realData, x_dim + 1, mem.kfRealFileName);
      } else {
        if (VERBOSE) {
          Serial.println("Error pulling info from simulation");
          while (1) {
            delay(1000);
          }
          break;
        }
      }
    }
  } else {
    sens.readSensors(currentData);
    currentData["intTime"] = currentData["time"] - prevSampleTime;
    float realData[x_dim + 1];
    realData[0] = currentData["time"];
    realData[1] = currentData["alt"];
    realData[2] = 0.0;
    realData[3] = currentData["linAccData2"];
    mem.logFloatData(realData, x_dim, mem.kfRealFileName);
  }
  prevSampleTime = currentData["time"];
}

/* Used for adding gaussian noise into the "measured" variables in the SIMULATION mode
   Is intended to evaluate the Kalman Filter Performance 
*/
float addGaussianNoise(float value, float mean, float stdDev) {
  // Generar dos números aleatorios uniformes en el rango (0, 1)
  float u1 = random(1, 10000) / 10000.0;  // Convertir a rango (0, 1)
  float u2 = random(1, 10000) / 10000.0;

  // Usar la transformación Box-Muller para generar números con distribución normal
  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);

  // Escalar el resultado al nivel deseado de varianza y media
  return value + z0 * stdDev + mean;
}

/*
    What has to happen every iteration so we are able to run the Kalman Filter
    1. Given currentData["intTime"] modify the F and Q matrices
    2. Locate the sensed data into z, the measurement vector
    3. Execute KF filter prediction
    4. Execute KF filter update
    5. Save data into kf_real (if it is a simulation), kf_measu, kf_output and kf_config file
  */
void KFStep() {
  //1
  static MatrixXf Q(x_dim, x_dim);
  Q = kf.set_Q_discrete_white_noise(x_dim, currentData["intTime"], process_variance);
  //kf.set_Q(Q); //not necessary when using set_Q_discrete_white_noise;
  //2
  static VectorXf z(z_dim);
  z(0) = currentData["alt"];
  z(1) = currentData["linAccData1"];
  //3
  kf.predict(currentData["intTime"]);
  //4 Update

  kf.update(z);
  //5
  kalmanState = kf.get_x();
  float configData[] = { (float)x_dim, (float)z_dim };
  int performanceSize = x_dim + 4;  //x_dim + likelihood + log_likelihood + NEES + rejection_threshold
  float performanceData[performanceSize];
  MatrixXf P_matrix = kf.get_P();

  for (int i = 0; i < x_dim; i++) {
    performanceData[i] = P_matrix(i, i);
  }

  performanceData[3] = kf.get_likelihood();
  performanceData[4] = kf.get_log_likelihood();
  if (IS_SIMULATION) {
    VectorXf ground_state(x_dim);
    ground_state << currentData["realAlt"],
      currentData["rawVel"],
      currentData["realAccData1"];
    performanceData[5] = kf.NEES_UPDATE(ground_state);
  } else {
    performanceData[5] = 0.0;
  }

  performanceData[6] = kf.get_rejectionThreshold();

  float measuData[] = { z(0), 0.0, z(1) };  //As we can only measure altitude and the velocity is a hidden variable
  float outputData[] = { currentData["time"], kalmanState(0), kalmanState(1), kalmanState(2) };


  mem.logFloatData(performanceData, performanceSize, mem.kfPerformanceFileName);
  mem.logFloatData(configData, 2, mem.kfConfigFileName);          //The 2 here is really hard coded NEEDS change
  mem.logFloatData(outputData, x_dim + 1, mem.kfOutputFileName);  //+1 counting for time
  mem.logFloatData(measuData, x_dim, mem.kfMeasuFileName);

  kfIteration++;
}

// Define the state transition function
VectorXf fx(const VectorXf& x, float dt) {
  VectorXf x_new(3);  // New state vector [position, velocity, acceleration]
  float velocity = x(1);

  float dragForce = 0.5 * Cd * rho * A * velocity * velocity * (velocity > 0 ? 1 : -1);
  x_new(2) = gravity - (dragForce / mass);
  x_new(1) = velocity + x_new(2) * dt;
  x_new(0) = x(0) + velocity * dt + 0.5 * x(2) * dt * dt;
  return x_new;
}

// Define the measurement function
VectorXf hx(const VectorXf& x) {
  MatrixXf H(z_dim, x_dim);
  H << 1, 0, 0,
    0, 0, 1;
  return H * x;
}


void messageAppend(float info, bool reset ) {
  static int messCounter = 0;
  if (reset) messCounter = 0;
  if (messCounter < messCoreLenght) messageCore[messCounter] = info;
  messCounter++;
}

void parseData() {
  /*
    Organizes the data in a float array for easier printing or radio-communicating it
    Also populates a more comprehensive array for local logging.
  */

  // Populate messageCore for LoRa (existing logic)
  for (int i = 0; i < messCoreLenght; i++) {
    messageCore[i] = 0.0;
  }
  if (IS_SIMULATION) {
    messageAppend(currentData["iter"], true);
  } else {
    messageAppend(cycleNumber, true);
  }
  messageAppend(currentData["linAccData0"]);
  messageAppend(currentData["linAccData1"]);
  messageAppend(currentData["linAccData2"]);
  messageAppend(currentData["euler0"]);
  messageAppend(currentData["euler1"]);
  messageAppend(currentData["euler2"]);
  messageAppend(currentData["alt"]);
  messageAppend((float)currentStage);
  messageAppend(gps.getVel());
  messageAppend(gps.getLatitude());
  messageAppend(gps.getLongitude());

  // Populate fullDataForLogging for local storage
  for (int i = 0; i < fullDataArraySize; i++) {
    fullDataForLogging[i] = 0.0; // Initialize with default
  }

  int idx = 0;
  fullDataForLogging[idx++] = IS_SIMULATION ? currentData["iter"] : cycleNumber;
  fullDataForLogging[idx++] = currentData["time"];
  fullDataForLogging[idx++] = currentData["temp"];
  fullDataForLogging[idx++] = currentData["prss"];
  fullDataForLogging[idx++] = currentData["alt"]; // Baro Alt
  fullDataForLogging[idx++] = currentData["deltaAlt"];
  fullDataForLogging[idx++] = currentData["humty"];
  fullDataForLogging[idx++] = currentData["maxAlt"];
  fullDataForLogging[idx++] = currentData["angVelData0"]; // Gyro X
  fullDataForLogging[idx++] = currentData["angVelData1"]; // Gyro Y
  fullDataForLogging[idx++] = currentData["angVelData2"]; // Gyro Z
  fullDataForLogging[idx++] = currentData["accData0"];    // Raw Accel X
  fullDataForLogging[idx++] = currentData["accData1"];    // Raw Accel Y
  fullDataForLogging[idx++] = currentData["accData2"];    // Raw Accel Z
  fullDataForLogging[idx++] = currentData["euler0"];      // Euler X
  fullDataForLogging[idx++] = currentData["euler1"];      // Euler Y
  fullDataForLogging[idx++] = currentData["euler2"];      // Euler Z
  fullDataForLogging[idx++] = currentData["linAccData0"]; // Linear Accel X
  fullDataForLogging[idx++] = currentData["linAccData1"]; // Linear Accel Y
  fullDataForLogging[idx++] = currentData["linAccData2"]; // Linear Accel Z
  fullDataForLogging[idx++] = (float)currentStage;
  fullDataForLogging[idx++] = gps.getLatitude();
  fullDataForLogging[idx++] = gps.getLongitude();
  fullDataForLogging[idx++] = gps.getAltitude(); // GPS Alt
  fullDataForLogging[idx++] = gps.getVel();    // GPS Speed
  fullDataForLogging[idx++] = (float)gps.getFixes(); // GPS Sats
  fullDataForLogging[idx++] = currentData["intTime"];

  // Kalman Filter Data (defaults to 0 if KF not active or values are not set)
  if (kalmanState.size() == x_dim) { // Ensure kalmanState is initialized
    fullDataForLogging[idx++] = kalmanState(0); // KF Pos
    fullDataForLogging[idx++] = kalmanState(1); // KF Vel
    fullDataForLogging[idx++] = kalmanState(2); // KF Acc
  } else {
    idx += 3; // Skip if KF state not ready
  }

  // Simulation-specific data
  if (IS_SIMULATION) {
    fullDataForLogging[idx++] = currentData["realAlt"];
    fullDataForLogging[idx++] = currentData["rawVel"];
    fullDataForLogging[idx++] = currentData["realAccData1"];
  } else {
    idx += 3; // Skip if not in simulation
  }
  // Ensure idx matches fullDataArraySize if all fields are populated,
  // or handle cases where some fields might be skipped.
  // The current logic fills up to `idx` elements. If `fullDataArraySize` is fixed,
  // remaining elements will be 0.0 from initialization.

  // Commented out old messageAppend calls for data not in messageCore
  // messageAppend(currentData["rawVel"]);
  // messageAppend(currentData["time"]);
  // messageAppend(currentData["temp"]);
  // messageAppend(currentData["accData0"]);
  // messageAppend(currentData["accData1"]);
  // messageAppend(currentData["accData2"]);
  // messageAppend(currentData["prss"]);
  // messageAppend(currentData["deltaAlt"]);
  // messageAppend(currentData["humty"]);
  // messageAppend(currentData["maxAlt"]);
  // messageAppend(gps.getFixes());
  // messageAppend(currentData["angVelData0"]);
  // messageAppend(currentData["angVelData1"]);
  // messageAppend(currentData["angVelData2"]);


  // Column Headers for Volta.txt (for reference, not written by this function):
  // 1.  Cycle/Iter
  // 2.  Time (s)
  // 3.  Temperature (C)
  // 4.  Pressure (hPa)
  // 5.  Altitude_Baro (m)
  // 6.  Delta_Altitude (m)
  // 7.  Humidity (%)
  // 8.  Max_Altitude (m)
  // 9.  Gyro_X (deg/s)
  // 10. Gyro_Y (deg/s)
  // 11. Gyro_Z (deg/s)
  // 12. RawAccel_X (m/s^2)
  // 13. RawAccel_Y (m/s^2)
  // 14. RawAccel_Z (m/s^2)
  // 15. Euler_X (deg)
  // 16. Euler_Y (deg)
  // 17. Euler_Z (deg)
  // 18. LinearAccel_X (m/s^2)
  // 19. LinearAccel_Y (m/s^2)
  // 20. LinearAccel_Z (m/s^2)
  // 21. Stage
  // 22. GPS_Latitude (deg)
  // 23. GPS_Longitude (deg)
  // 24. GPS_Altitude (m)
  // 25. GPS_Speed (m/s)
  // 26. GPS_Satellites
  // 27. Interval_Time (s)
  // 28. KF_Position (m)
  // 29. KF_Velocity (m/s)
  // 30. KF_Acceleration (m/s^2)
  // 31. Sim_RealAltitude (m)
  // 32. Sim_RawVelocity (m/s)
  // 33. Sim_RealAccelY (m/s^2)
}

void dynamicDelay() {
  /*
    Delays the running of the code without actually stopping it
  */
  while (int(millis() - lastTime) < sampleDelay && !(IS_SIMULATION)) {
    delay(1);
  }
  lastTime = millis();
  cycleNumber++;
}

/*
  * From here on, every init function initializes an specific stage of the flight and its only called once.
  * Every termination function terminates a stage, meaning, it detects that the stage termination condiction was matched to 
  asign the current stage as the next one in order.
*/

void startUpInit() {
  /*  
    Initialize board's neccesary systems
    It only works the first time it is ever called so sensors are not accidently rebooted
    This code would usually go in the setup function but as it is part of a rocket-stage is better to create a function for it
  */
  static bool startupInitializer = true;  //Variable that protects this function from beeing called more than once
  digitalWrite(RLED, HIGH);
  if (startupInitializer) {
    if (VERBOSE) {
      Serial.println("Starting all systems...");
    }

    /*
      GPS begin can be tried 3 times and if none of them really work it concludes that it was not found.
    */
    const int MAX_GPS_RETRIES = 3;

    bool gpsInitialized = false;  // Flag to track success

    for (int attempt = 0; attempt < MAX_GPS_RETRIES; attempt++) {
      if (gps.begin()) {
        gpsInitialized = true;
        break;  // Exit loop if GPS initializes successfully
      }
      delay(100);  // Wait 100ms before retrying
    }

    // If still not initialized after 3 tries, enter error loop
    if (!gpsInitialized) {
      while (1) {
        if (VERBOSE) {
          Serial.println("GPS NOT FOUND");
        }
        delay(1000);
      }
    }


    if (!lora.begin()) {
      while (1) {
        if (VERBOSE) {
          Serial.println("LORA NOT CONNECTED");
        }
        delay(1000);
      }
    }
    if (!sens.begin()) {
      while (1) {
        if (VERBOSE) {
          Serial.println("IMU OR BARO NOT FOUND");
        }
        delay(1000);
      }
    }
    /*
      Setup sensor modes:
      Low barometer for the 0 height reference pressure
    */
    sens.setBaroMode(LOW_RATE);
    
    sens.setIMUMode(IMUONLY_NDOF);

    if (IS_SIMULATION) {  //Initialize simulation if we are running one
      randomSeed(2);
      if (!mem.begin(CS_SD)) {
        while (1) {
          if (VERBOSE) {
            Serial.println("COULDN'T REACH SD CARD");
          }
          delay(1000);
        }
      }
      if (VERBOSE) {
        Serial.println("This is a simulation");
      }
      simDataColumnNumber = mem.startSimulationData();
      if (simDataColumnNumber == 0) {
        if (VERBOSE) {
          Serial.println("Simulation file not found");
        }
      } else if (simDataColumnNumber == -1) {
        if (VERBOSE) {
          Serial.println("Simulation file is empty");
        }
      } else if (simDataColumnNumber >= 1) {
        if (VERBOSE) {
          Serial.print("Simulation file initialized, # columns: ");
          Serial.println(simDataColumnNumber);
        }
      }
    } else {
      if (!mem.begin(CS_FLASH)) {
        while (1) {
          if (VERBOSE) {
            Serial.println("FLASH NOT FOUND");
          }
          delay(1000);
        }
      }
      lora.transmitString("Taking reference pressure");
      sens.setReferencePressure();
    }
    pyro.begin();
    if (VERBOSE) {
      Serial.println("All systems initialized");
    }

    /*If we are manually calibrating the device we want to save new calibration parameters.
      If we don't make the manual calibration then the saved calibration parameters of the last calibration will be used
      It is recomended to make a manual calibration in every new integration step or change in the surroundings of the flight computer
    */


    if (manualCalibration) {
      while (!sens.saveCalibration()) {
        sens.checkCalibrationStatus();
        delay(500);
        Serial.print("Shake and move the device: ");
   
      }
      
    }
    
    //So we dont re-initialize it:
    startupInitializer = false;
  }
}

void startupTermination() {
  /*
    Terminate conditions:
    * startup when GPS has more than 5 sat fixed
    * IMU calibration is complete
    * 500 pressure samples where taken    
    * is a simulation
  */
  if (groundConfirmation) {
    if ( groundConfirmation || IS_SIMULATION) {
      //
      if (VERBOSE) {
        Serial.println("Startup phase complete - all systems ready for flight");
      }
      currentStage = IDLE;  //Move to the next stage (Idle, naturally)
    } else {
      String mess = "Waiting enough gps fixes: " + String(gps.getFixes()) + " currently"; //change this to the number of fixes needed ())
      lora.transmitString(mess);
      delay(1000);
    }
  } else {
    lora.transmitString("Waiting for ground confirmation");
  }
}

/**
 * @brief Initializes system parameters and sensor settings for an idle state.
 *
 * This function is intended to be called when the system enters an idle or standby
 * mode. It configures various settings to optimize for this state, such as adjusting
 * sensor sampling rates and providing visual feedback.
 *
 * Key configurations set by this function:
 * - `sampleDelay`: Set to 500 (presumably milliseconds). This will affect how frequently
 *   sensor data is sampled by other parts of the system that use this variable.
 * - RLED (Red LED): Turned LOW (off), likely to indicate the system is in an idle state.
 * - Verbose Logging: If `VERBOSE` is true, a confirmation message "Idle mode initialized"
 *   is printed to the Serial output.
 * - Barometer Mode: The barometer (`sens`) is set to `LOW_RATE`. This typically means
 *   the barometer will sample less frequently, reducing power consumption and data output
 *   when high-frequency pressure readings are not required.
 * - IMU Mode: The IMU (`sens`) is set to `HIGH_RATE`. This might seem counter-intuitive
 *   for an idle mode. However, the original code includes a comment:
 *   "We can't use the Fusion mode of the BNO055 as it's maximum acceleration is 4g's
 *   and the rocket is expected to feel up to 6g on launch".
 *   This implies that even in an idle state (e.g., pre-launch), the IMU is kept at a
 *   high data rate (and potentially a non-fusion mode like the commented-out `OPERATION_MODE_AMG`)
 *   to ensure it can accurately capture high-acceleration events if the system transitions
 *   out of idle unexpectedly or upon launch. This avoids missing critical data due to
 *   sensor limitations in fusion modes at high G-forces.
 *
 * One-Time Execution:
 * This function uses a `static bool idleInitializer` flag. This ensures that the
 * initialization logic within the `if (idleInitializer)` block is executed only
 * once, the first time `idleInit()` is called. Subsequent calls to `idleInit()`
 * will do nothing, preventing repeated re-initialization.
 *
 * Implications for the rest of the code:
 * - The `sampleDelay` value directly influences the timing of data acquisition loops
 *   elsewhere in the program.
 * - The sensor modes (`LOW_RATE` for baro, `HIGH_RATE` for IMU) determine the
 *   responsiveness, data fidelity, and power consumption characteristics of these sensors.
 *   Other parts of the code reading from `sens` will receive data according to these settings.
 * - The state of RLED can be used as a visual diagnostic for the system's current mode.
 * - The commented-out lines `//lora.setToSleep();` and `//sens.putToSleep();` suggest
 *   that further power optimization by putting LoRa module and other sensor components
 *   into a deeper sleep state might be intended or was previously considered for this
 *   idle mode. This could be relevant for battery-powered applications.
 *
 * @note This function should be called when the system is intended to be quiescent
 *       but ready to transition to an active state, potentially with specific sensor
 *       configurations pre-set for that transition (e.g., high-rate IMU for launch).
 */

void idleInit() {
  // This function configures system parameters for the IDLE state.
  // It's designed to run only once when entering IDLE to prevent re-initialization.
  static bool idleInitializer = true;
  if (idleInitializer) {

    // --- General Settings for IDLE Mode ---

    // Variable: sampleDelay
    // Purpose: Sets the interval (in milliseconds) between sensor data samples.
    // Current Value: 500 ms
    // Impact: Affects data acquisition rate. A higher value means less frequent samples,
    //         which can reduce processing load and power consumption during IDLE.
    // Operator Note: Adjust the '500' below if a different sampling frequency is desired for IDLE.
    //sampleDelay = 0;

    // Variable: RLED (Red LED)
    // Purpose: Provides a visual status indication for the system.
    // Current State: LOW (OFF)
    // Impact: The Red LED is turned OFF to visually signify that the system is out Startup mode.
    // Impact: The Green LED is turned ON to visually signify that the system is in IDLE mode.
    // Operator Note: Change 'LOW' to 'HIGH' if an illuminated LED is preferred to indicate IDLE.
    digitalWrite(RLED, LOW);

    digitalWrite(GLED, HIGH);

    if (VERBOSE) {
      Serial.println("Idle mode initialized");
    }

    // --- Sensor Configuration for IDLE Mode ---
    // These settings aim to balance power consumption with readiness for transitioning to active states.

    // Sensor: Barometer (accessed via 'sens' object)
    // Setting: Barometer Mode
    // Current Value: LOW_RATE
    // Purpose: Reduces the barometer's sampling frequency. This minimizes data output and power usage
    //          when high-frequency pressure readings are not critical (i.e., during IDLE).
    // Operator Note: If more frequent pressure data is needed during IDLE, change 'LOW_RATE'
    //                to 'MID_RATE' or 'HIGH_RATE' (refer to sensor library for specifics).
    sens.setBaroMode(LOW_RATE);

    // Sensor: IMU (accessed via 'sens' object, e.g., BNO055)
    // Setting: IMU Mode
    // Current Value: HIGH_RATE
    // Rationale: The IMU is kept at a high data rate even in IDLE. This is a precaution:
    //            standard fusion modes of the BNO055 (like NDOF) might have a maximum acceleration
    //            limit of around 4g. The rocket is expected to experience up to ~6g on launch.
    //            Setting a HIGH_RATE mode (often a non-fusion or basic IMU data mode) ensures
    //            the sensor can capture such high-G events accurately if the system transitions
    //            rapidly from IDLE to an active state (e.g., launch).
    // Operator Note: If a different IMU behavior or a specific fusion/non-fusion mode is desired
    //                during IDLE, change 'HIGH_RATE'. Consult the sensor library for available modes
    //                (e.g., NDOF, IMUONLY_NDOF, OPERATION_MODE_AMG).
    //sens.setIMUMode(HIGH_RATE);

    /*
    // Example: Setting IMU to a specific non-fusion mode (Accelerometer, Magnetometer, Gyroscope).
    // This line is currently commented out.
    // Operator Note: Uncomment 'sens.imu.setMode(OPERATION_MODE_AMG);' if raw sensor data
    //                without on-board fusion is preferred during the IDLE state.
    // sens.imu.setMode(OPERATION_MODE_AMG);
    */

    // --- Optional Power Saving Measures (Currently Commented Out) ---
    // These lines can be enabled for applications where minimizing power consumption
    // during extended IDLE periods is critical (e.g., battery-powered operation).

    // Device: LoRa Module
    // Action: Put to Sleep
    // Operator Note: Uncomment 'lora.setToSleep();' to enable sleep mode for the LoRa module
    //                during IDLE. Ensure it's woken up before needing to transmit/receive.
    // // lora.setToSleep();

    // Device: Main Sensor Unit (accessed via 'sens' object)
    // Action: Put to Sleep
    // Operator Note: Uncomment 'sens.putToSleep();' to enable sleep mode for the primary sensors
    //                during IDLE. Ensure sensors are woken up before active data acquisition.
    // // sens.putToSleep();

    idleInitializer = false; // Flag to ensure this initialization logic runs only once.
  }
}

void idleTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is higer than 5g
    TODO: DO NOT USE z acceleration but net acceleration (can't be done because max g's on fusion mode is 4g's) and higher altitude.
  */
  if (currentData["accData1"] - 9.81 > (9.81 * 2)) {  //we passed the net 2g threshold
    if (VERBOSE) {
      Serial.println("Idle terminated");
    }
    currentStage = BOOSTING;
  }
}

void boostInit() {
  static bool boostInitializer = true;
  if (boostInitializer) {
    /* Set boosting characteristics */
    if (VERBOSE) {
      Serial.println("Boosting initialized");
    }
    sens.setBaroMode(HIGH_RATE);
    sens.restoreCalibration();  //It is very important to restore calibration as it allows correct functioning of fusion mode.
    //sens.setIMUMode(IMUONLY_NDOF);
    boostInitializer = false;
  }
}

void boostTermination() {
  /*Check for stage finalization condition:
    * Acceleration along the z axis is lower than 1.2g
  */
  if (currentData["linAccData1"] < 0) {
    if (VERBOSE) {
      Serial.println("Boosting terminated");
    }
    currentStage = COASTING;
  }
}

void coastInit() {
  static bool coastInitializer = true;
  if (coastInitializer) {
    kfIteration = 0;
    /*
      * Initialize Kalman Filter for state tracking.
    */
    VectorXf initial_state(x_dim);
    MatrixXf initial_covariance(x_dim, x_dim);
    initial_state << currentData["alt"], 300, -9.81;
    initial_covariance << 500, 0, 0,  //could be all zeros as we know for sure that the rocket is in the ground and idleing
      0, 5000, 0,
      0, 0, 500;
    kf.initialize_state(initial_state, initial_covariance);
    MatrixXf H(z_dim, x_dim);
    MatrixXf R(z_dim, z_dim);
    H << 1, 0, 0,
      0, 0, 1;
    R << pos_measurement_variance, 0,
      0, acc_measurement_variance;
    kf.set_fx(fx);
    kf.set_hx(hx);
    kf.set_H(H);
    kf.set_R(R);
    /* Set coasting characteristics */
    if (VERBOSE) {
      Serial.println("Coasting initialized");
    }
    sens.setBaroMode(MID_RATE);
    sens.restoreCalibration();  //It is very important to restore calibration as it allows correct functioning of fusion mode.
    //sens.setIMUMode(IMUONLY_NDOF);
    coastInitializer = false;
  }
}

void coastTermination() {
  /*
    Check for stage finalization condition:
    * If we trust the Kalman Filter: the velocity now is negative
    * If we do not trust the Kalman Filter: The distance from the apogee is more than 3 meters for 5 iterations in a row  
  */
  static int counter = 0;
  static bool terminate = false;
  if (kfIteration > 5) {
    trustKalman = true;
  }
  if (trustKalman) {
    if (kalmanState(1) < 0.0) {  //(1) is velocity
      terminate = true;
    }
  } else {
    if ((currentData["maxAlt"] - currentData["alt"]) > 3) counter++;
    else counter = 0;
    if (counter > 5) {
      terminate = true;
    }
  }
  if (terminate) {
    if (VERBOSE) {
      Serial.println("Coasting terminated");
    }
    currentStage = DROGUEDESCENT;
  }
}

void drogueInit() {
  static bool drogueInitializer = true;
  if (drogueInitializer) {
    Cd = Cd_parachute_drogue + Cd_rocket;
    kfIteration = 0;
    /* Set drogue descent characteristics*/
    if (VERBOSE) {
      Serial.println("Drogue descent initialized");
    }
    sens.setBaroMode(MID_RATE);
    //sens.setIMUMode(IMUONLY_IMU);
    pyro.firePyro(1, 'a');
    pyro.firePyro(1, 'b');
    pyro.firePyro(2, 'a');
    pyro.firePyro(2, 'b');
    drogueInitializer = false;
  }
}

void drogueTermination() {
  /*
    Check for stage finalization condition:
    * The altitude is lower than 1200mts
  */
  if (currentData["alt"] < 1200) {
    if (VERBOSE) {
      Serial.println("Drogue descent terminated");
    }
    currentStage = MAINDESCENT;
  }
}

void mainDescentInit() {
  static bool mainDescentInitializer = true;
  if (mainDescentInitializer) {
    /* Main descent characteristics */
    Cd = Cd_parachute_main + Cd_rocket;
    kfIteration = 0;
    trustKalman = false;
    if (VERBOSE) {
      Serial.println("Main descent initialized");
    }
    sens.setBaroMode(HIGH_RATE);
    //sens.setIMUMode(HIGH_RATE);
    pyro.firePyro(4, 'a');
    pyro.firePyro(4, 'b');
    pyro.firePyro(5, 'a');
    pyro.firePyro(5, 'b');
    mainDescentInitializer = false;
  }
}

void mainDescentTermination() {
  static int counterDT = 0;
  static bool terminate = false;

  /*Check for stage finalization condition:
    * If we trust the Kalman Filter: velocity is almost 0
    * If we do NOT trust the Kalman Filter: If altitude is lower than 10 mts
  */
  if (kfIteration > 5) {
    trustKalman = true;
  }
  if (trustKalman) {
    if (abs(kalmanState(1)) < 1.0) {
      terminate = true;
    }
  } else {
    if (currentData["alt"] < 10.0) counterDT++;
    else counterDT = 0;
    if (counterDT >= 3) {
      terminate = true;
    }
  }
  if (terminate) {
    if (VERBOSE) {
      Serial.println("Main descent terminated");
    }
    currentStage = TOUCHDOWN;
  }
}

void touchDownInit() {
  static bool touchDownInit = true;
  if (touchDownInit) {
    /* Touch down descent characteristics */
    if (VERBOSE) {
      Serial.println("Touch down");
    }
    sens.setBaroMode(MID_RATE);
    sens.setIMUMode(HIGH_RATE);
    sens.imu.setMode(OPERATION_MODE_AMG);
    touchDownInit = false;
  }

  // Turn Off cameras
  digitalWrite(CAMERA_PIN, LOW);

}

/*
  Esta funcion tiene uso especialmente cuando se está en IDLE en el launchpad para enviar comandos a la computadora a bordo
  Esta funcion recive ese comando y lleva a cabo trasmisiones respectivas a cada comando
  */
void checkCommand() {
  lora.checkReceive(); // This should read the first byte into lora.lastCommand and potentially buffer the rest
  byte command = lora.lastCommand;
  
  if (command == 0x01) {
    groundConfirmation = true;
    if (VERBOSE) {
      Serial.println("Ground confirmed flight readiness");
    }
        lora.lastCommand = 0x00;
  } else if (command == 0x04) {
    // Request pyro continuity status
    if (VERBOSE) {
      Serial.println("Pyro continuity status requested by ground.");
    }
//    lora.transmitContinuity(continuityPyros); // Send continuity data
    lora.lastCommand = 0x00; // Command handled
  } else if (command == 0x05) {
    if (VERBOSE) {
      Serial.println("Ejection chamber info requested (Not Implemented)");
      // Placeholder: Send some status if implemented later
    }
        lora.lastCommand = 0x00;
  } else if (command == 0x06) {
    if (VERBOSE) {
      Serial.println("Turning cameras On...");
    }
    // Turn on the cameras
    digitalWrite(CAMERA_PIN, HIGH);
        lora.lastCommand = 0x00;
  } else if (command == 0x07) {
    if (VERBOSE) {
      Serial.println("Turning cameras Off...");
    }
    // Turn off the cameras
    digitalWrite(CAMERA_PIN, LOW);
        lora.lastCommand = 0x00;
  } else if (command == 0x08) {
    // Command to request LoRa frequency change (Step 1 of handshake)
    String freqStr = LoRa.readString(); // Read the frequency string payload (should be in Hz)
    
    if (freqStr.length() > 0) {
        long newFrequency = atol(freqStr.c_str()); // Convert string to long (Hz)

        if (VERBOSE) {
            Serial.print("Received frequency change request (0x08). Requested frequency: ");
            Serial.print(newFrequency);
            Serial.println(" Hz");
        }
        
        // Validate the frequency range (915 MHz to 930 MHz for rocket)
        if (newFrequency >= 915E6 && newFrequency <= 930E6) { // Adjusted range check
            if (VERBOSE) {
                Serial.print("Frequency is valid. Sending confirmation (0x09) and storing pending change to: ");
                Serial.println(newFrequency);
            }

            // 1. Send confirmation back to GS (Step 2 of handshake)
            LoRa.beginPacket();
            LoRa.write(ROCKET_FREQ_CHANGE_CONFIRM); // Send confirmation ID 0x09
            LoRa.print(freqStr);                   // Send back the frequency string
            LoRa.endPacket();
            
            // 2. Store the frequency, but DO NOT change yet
            pendingRocketFreqChangeHz = newFrequency;
            
        } else {
            if (VERBOSE) {
                Serial.print("Invalid frequency requested: ");
                Serial.print(newFrequency);
                Serial.println(" Hz. Ignoring request. Must be between 915MHz and 930MHz.");
            }
            pendingRocketFreqChangeHz = 0; // Ensure no pending change if invalid
        }
        
    } else {
        if (VERBOSE) {
            Serial.println("Received frequency change command (0x08) but payload was empty.");
        }
        pendingRocketFreqChangeHz = 0; // Ensure no pending change
    }
    
    lora.lastCommand = 0x00; // Clear command
        
  } else if (command == CMD_EXECUTE_FREQ_CHANGE) { // Command 0x0A (Step 4 of handshake)
      if (VERBOSE) {
          Serial.print("Received Execute Frequency Change command (0x0A). ");
      }
      // Check if a frequency change was actually pending
      if (pendingRocketFreqChangeHz != 0) {
          if (VERBOSE) {
              Serial.print("Executing pending frequency change to: ");
              Serial.println(pendingRocketFreqChangeHz);
          }
          // Execute the frequency change
          LoRa.setFrequency(pendingRocketFreqChangeHz); 
          
          if (VERBOSE) {
              Serial.println("Rocket LoRa frequency changed."); 
          }
          // Clear the pending change
          pendingRocketFreqChangeHz = 0; 
      } else {
          if (VERBOSE) {
              Serial.println("No frequency change was pending. Ignoring command.");
          }
      }
      lora.lastCommand = 0x00; // Clear command
  } else if (command == CMD_DELETE_VOLTA_FILE) {
    if (VERBOSE) {
      Serial.println("Command received: Delete Volta.txt file.");
    }

    mem.checkAndDeleteFile(mem.dataFileName); // Delete the file if it exists
    
    lora.lastCommand = 0x00; // Command handled
  }   
    
}

void transmitDataDelayed() {
  static long lastTransmit = 0;
  if ((millis() - lastTransmit) > 0) {
    lora.transmitData(messageCore, messCoreLenght);
    lastTransmit = millis();
  }
}

// For Writer Code 
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