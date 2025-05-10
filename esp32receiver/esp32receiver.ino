#include <SPI.h>
#include <LoRa.h>

// LoRa configuration

// SCK: Serial Clock pin for SPI communication
#define LORA_SCK 18
// MISO: Master In Slave Out pin for SPI communication
#define LORA_MISO 19  
// MOSI: Master Out Slave In pin for SPI communication
#define LORA_MOSI 23
// SS: Slave Select pin for SPI communication
#define LORA_SS 5
// RST: Reset pin for LoRa module
#define LORA_RST 14
// DIO0: Digital IO pin used for LoRa interrupts
#define LORA_DIO0 26

// Command codes
#define CMD_GROUND_CONFIRM 0x01
#define CMD_PYRO_STATUS 0x04
#define CMD_EJECTION_INFO 0x05 // Not implemented on rocket
#define CMD_CAMERA_ON 0x06
#define CMD_CAMERA_OFF 0x07
#define CMD_SET_FREQUENCY 0x08 // GS sends this to rocket (Step 1)
#define CMD_ACK 0x07             // Rocket sends this general ACK
#define ROCKET_FREQ_CHANGE_CONFIRM 0x09 // Rocket sends this confirmation (Step 2)
#define CMD_EXECUTE_FREQ_CHANGE 0x0A // GS sends this to execute change (Step 3)

// Special code to indicate waiting for frequency input via Serial
#define AWAIT_FREQUENCY_INPUT 0xFF

// onAwait: Tracks current command state
//          0x00: Idle, waiting for user command
//          AWAIT_FREQUENCY_INPUT (0xFF): Waiting for user to type frequency
//          CMD_SET_FREQUENCY (0x08): Command 0x08 sent, waiting for ROCKET_FREQ_CHANGE_CONFIRM (0x09)
//          Other CMD_ values: Command sent, waiting for CMD_ACK (0x07)
byte onAwait = 0x00;

// Delay between retransmission attempts in milliseconds (only for 0x08)
int insistDelay = 500; // Increased delay slightly

// Timestamp of last transmission, used for timing retries
unsigned long lastTransmit = 0;

// Variable to store the frequency string (in Hz) to be sent with 0x08
String frequencyStringToSend = "";

// Variables for scheduling GS frequency change after sending 0x0A
long pendingGsFreqChangeHz = 0; 
unsigned long gsFreqChangeExecuteTime = 0;
const unsigned long GS_CHANGE_DELAY_MS = 500; // Wait 500ms after sending 0x0A before GS changes

// Variables for repeatedly sending Execute command (0x0A)
bool isSendingExecuteCmd = false;
unsigned long executeCmdSendUntil = 0;
unsigned long lastExecuteCmdSentTime = 0;
const unsigned long EXECUTE_CMD_SEND_DURATION_MS = 4000; // Send 0x0A for 1.5 seconds
const unsigned long EXECUTE_CMD_INTERVAL_MS = 150;     // Send 0x0A every 150ms during the duration

void setup() {
  // Initialize Serial communication at 9600 baud rate
  Serial.begin(115200);
  // Wait for Serial connection to be established
  while (!Serial)
    ;

  Serial.println("LoRa Receiver Station");

  // Configure LoRa module with defined pins for SPI communication
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  // Initialize LoRa at 915MHz frequency (North American band) - Initial Frequency
  long initialFrequency = 915E6;
  if (!LoRa.begin(initialFrequency)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  Serial.print("LoRa initialized at: ");
  Serial.print(initialFrequency / 1E6);
  Serial.println(" MHz");

  // Configure LoRa radio parameters:
  // - Sync word: 0xAF (helps filter unwanted packets)
  // - Spreading Factor: 10 (tradeoff between range and data rate)
  // - Signal Bandwidth: 250kHz (wider bandwidth = faster data rate but shorter range)
  LoRa.setSyncWord(0xAF);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(250E3);

  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║        LoRa Receiver Station         ║");
  Serial.println("║----------------------------------    ║");
  Serial.println("║ ✓ Serial Communication Established   ║");
  Serial.println("║ ✓ LoRa Module Initialized            ║");
  Serial.println("║ ✓ Radio Parameters Configured        ║");
  Serial.println("║   - Sync Word: 0xAF                  ║");
  Serial.println("║   - Spreading Factor: 10             ║");
  Serial.println("║   - Bandwidth: 250kHz                ║");
  Serial.println("║                                      ║");
  Serial.println("║     Ready to Receive Data!           ║");
  Serial.println("╚══════════════════════════════════════╝");
}

void loop() {
  // --- Execute Scheduled GS Frequency Change ---
  if (pendingGsFreqChangeHz != 0 && !isSendingExecuteCmd && millis() >= gsFreqChangeExecuteTime) { // Only change if not sending 0x0A and time is up
      Serial.print("Executing scheduled GS frequency change to: ");
      Serial.print(pendingGsFreqChangeHz / 1E6); // Print in MHz
      Serial.println(" MHz");
      
      // Change GS LoRa frequency
      LoRa.setFrequency(pendingGsFreqChangeHz);
      
      Serial.println("Local GS LoRa frequency changed.");
      pendingGsFreqChangeHz = 0; // Clear pending change
      gsFreqChangeExecuteTime = 0;
  }

  // --- Repeatedly Send Execute Command (0x0A) if in that phase ---
  if (isSendingExecuteCmd) {
      if (millis() >= executeCmdSendUntil) {
          // Time to stop sending 0x0A
          isSendingExecuteCmd = false;
          // Schedule the local GS frequency change (using the already stored pendingGsFreqChangeHz)
          gsFreqChangeExecuteTime = millis() + GS_CHANGE_DELAY_MS;
          Serial.print("Finished sending 0x0A. Scheduled local GS frequency change in ");
          Serial.print(GS_CHANGE_DELAY_MS);
          Serial.println(" ms.");
      } else {
          // Continue sending 0x0A at intervals
          if (millis() - lastExecuteCmdSentTime >= EXECUTE_CMD_INTERVAL_MS) {
              Serial.println("Sending Execute command (0x0A)...");
              LoRa.beginPacket();
              LoRa.write(CMD_EXECUTE_FREQ_CHANGE);
              LoRa.endPacket();
              lastExecuteCmdSentTime = millis();
          }
      }
  }

  // --- Check for Incoming Packets ---
  if (LoRa.parsePacket()) {
    byte dataID = LoRa.read(); // Read the first byte (ID)
    
    Serial.print(dataID);
    Serial.print(": ");
    
    // --- Handle Rocket Confirmation for Frequency Change (0x09) --- Step 2 Received
    if (dataID == ROCKET_FREQ_CHANGE_CONFIRM) {
        String confirmedFreqStr = "";
        while(LoRa.available()) {
            confirmedFreqStr += (char)LoRa.read();
        }
        
        Serial.print("Received Frequency Change Confirmation (0x09) from rocket. Confirmed Frequency: ");
        Serial.print(confirmedFreqStr);
        Serial.println(" Hz");

        // Check if we were actually waiting for this confirmation (sent 0x08)
        if (onAwait == CMD_SET_FREQUENCY) {
            long confirmedFrequencyHz = atol(confirmedFreqStr.c_str());

            // Optional: Validate again if the confirmed frequency matches what we intended to send
            // and is within the valid range (905-930 MHz for GS)
            if (confirmedFrequencyHz >= 905E6 && confirmedFrequencyHz <= 930E6 && confirmedFreqStr == frequencyStringToSend) {
                 Serial.println("Confirmation matches request. Starting to send Execute command (0x0A) repeatedly.");
                 
                 // Store frequency for later local change
                 pendingGsFreqChangeHz = confirmedFrequencyHz; 
                 
                 // Start the process of sending 0x0A repeatedly
                 isSendingExecuteCmd = true;
                 executeCmdSendUntil = millis() + EXECUTE_CMD_SEND_DURATION_MS;
                 lastExecuteCmdSentTime = 0; // Send first one immediately in the next loop iteration

                 // Reset state: Handshake proceeding
                 onAwait = 0x00; 
                 frequencyStringToSend = ""; // Clear stored frequency
            } else {
                 Serial.print("Error: Confirmed frequency ");
                 Serial.print(confirmedFreqStr);
                 Serial.println(" Hz does not match request or is invalid.");
                 Serial.println("Aborting frequency change. Resetting state.");
                 // Reset state: Handshake failed
                 onAwait = 0x00; 
                 frequencyStringToSend = ""; 
            }
        } else {
            Serial.println("Received frequency change confirmation (0x09), but wasn't expecting it (State != 0x08). Ignoring.");
        }
    } 
    // --- Handle General Acknowledgment (0x07) ---
    else if (dataID == CMD_ACK) {
        Serial.print("Received General Acknowledgment (0x07) for command 0x");
        Serial.println(onAwait, HEX);

        // Ignore ACKs if we are in the middle of the frequency change handshake (waiting for 0x09)
        // Also ignore if we are busy sending the execute command (0x0A)
        if (onAwait == CMD_SET_FREQUENCY || isSendingExecuteCmd) {
            Serial.println("Ignoring general ACK (0x07) during frequency change handshake/execution.");
        } 
        // For any other command we were waiting for, the general ACK is sufficient.
        else if (onAwait != 0x00 && onAwait != AWAIT_FREQUENCY_INPUT) {
            Serial.println("Command acknowledged. Resetting state.");
            onAwait = 0x00; // Reset state, command successful
            frequencyStringToSend = ""; // Should be clear anyway
        } else {
             Serial.println("Received unexpected general ACK (0x07). Ignoring.");
        }
    }
    // --- Handle Telemetry and Other Data ---
    else if (dataID == 0x00 || dataID == 0x03) { // Core Telemetry or GPS
      while (LoRa.available()) {
        String received = LoRa.readString();
        Serial.print(received);
      }
    } else if(dataID == 0x02) { // Secondary Telemetry
      while (LoRa.available()) {
        String received = LoRa.readString();
        Serial.print(received);
      }
    } else if (dataID == 0x04) { // Pyro Status
      bool pyroContinuity[10];  

      for (int i = 0; i < 10; i++) {
        byte receivedByte = LoRa.read();          
        pyroContinuity[i] = (receivedByte == 1);  
      }

      for (int i = 0; i < 10; i++) {
        Serial.print(pyroContinuity[i]);
        Serial.print(" ");
      }
    } else if (dataID == 0x06) { // Reserved
      // Currently no implementation
    } else {
      Serial.print("Received unknown data ID: 0x"); 
      Serial.print(dataID, HEX);
      Serial.print(" RSSI: ");
      Serial.print(LoRa.rssi());
      while(LoRa.available()) { LoRa.read(); } 
    }
    Serial.println(); // End the line after processing any message type
  }

  // --- Handle User Input and Command Sending/Retransmission ---
  // Only handle user input/retransmissions if NOT busy sending 0x0A
  if (!isSendingExecuteCmd) {
      // State: Waiting for user to type frequency
      if (onAwait == AWAIT_FREQUENCY_INPUT) {
          if (Serial.available() > 0) {
              String freqStrMHz = Serial.readStringUntil('\n');
              freqStrMHz.trim(); 

              if (freqStrMHz.length() > 0) {
                  float requestedFrequencyMHz = atof(freqStrMHz.c_str()); 
                  long requestedFrequencyHz = (long)(requestedFrequencyMHz * 1E6);

                  if (requestedFrequencyHz >= 905E6 && requestedFrequencyHz <= 930E6) {
                      frequencyStringToSend = String(requestedFrequencyHz); 
                      Serial.print("Frequency ");
                      Serial.print(freqStrMHz);
                      Serial.print(" MHz (");
                      Serial.print(frequencyStringToSend);
                      Serial.println(" Hz) is valid. Sending command 0x08 to rocket.");
                      
                      onAwait = CMD_SET_FREQUENCY; 

                      LoRa.beginPacket();
                      LoRa.write(onAwait); 
                      LoRa.print(frequencyStringToSend); 
                      LoRa.endPacket();
                      Serial.println("Command 0x08 sent. Waiting for confirmation 0x09 from rocket...");
                      lastTransmit = millis(); 
                  } else {
                      Serial.print("Error: Requested frequency ");
                      Serial.print(freqStrMHz);
                      Serial.println(" MHz is outside the allowed range (905-930 MHz).");
                      Serial.println("Command NOT sent. Please try again.");
                      onAwait = 0x00; 
                      frequencyStringToSend = ""; 
                  }
              } else {
                  Serial.println("Invalid frequency entered. Please enter frequency in MHz (905 - 930).");
                  onAwait = 0x00; 
                  frequencyStringToSend = ""; 
              }
          }
      } 
      else if (onAwait == CMD_SET_FREQUENCY) {
          if (millis() - lastTransmit > insistDelay) {
              Serial.print("No confirmation (0x09) received. Retransmitting command 0x08 with frequency ");
              Serial.print(frequencyStringToSend);
              Serial.print(" Hz..."); 
              
              LoRa.beginPacket();
              LoRa.write(onAwait); 
              LoRa.print(frequencyStringToSend); // Ensure frequency string is sent on retransmit
              LoRa.endPacket();
              Serial.println(" Sent.");
              lastTransmit = millis(); 
          }
      } 
      else if (onAwait != 0x00 && onAwait != AWAIT_FREQUENCY_INPUT) {
          if (millis() - lastTransmit > insistDelay) { 
              Serial.print("No acknowledgment (0x07) received. Retransmitting command 0x"); 
              Serial.println(onAwait, HEX);
              
              LoRa.beginPacket();
              LoRa.write(onAwait); 
              LoRa.endPacket();
              lastTransmit = millis(); 
          }
      }
      else { 
          if (pendingGsFreqChangeHz == 0) { 
              byte commandToSend = getCommand(); 
              
              if (commandToSend != 0x00) {
                  if (commandToSend == AWAIT_FREQUENCY_INPUT) {
                      onAwait = AWAIT_FREQUENCY_INPUT;
                  } else {
                      onAwait = commandToSend; 
                      LoRa.beginPacket();
                      LoRa.write(onAwait);
                      LoRa.endPacket();
                      Serial.print("Command sent: 0x"); Serial.println(onAwait, HEX);
                      Serial.println("Waiting for acknowledgment (0x07)...");
                      lastTransmit = millis(); 
                  }
              }
          }
      }
  } // End of if (!isSendingExecuteCmd)
}

byte getCommand() {
  if (Serial.available()) {
    char incomingChar = Serial.read();  
    
    switch(incomingChar) {
      case 's':
        return 0x01;
        
      case 'p': 
        return 0x04;
        
      case 'c':
        return 0x05;
        
      case 'o':
        return 0x06;

      case 'f':
        return 0x07;

      case 'l':
        Serial.print("Enter new LoRa frequency in MHz: "); 
        return AWAIT_FREQUENCY_INPUT; 
        
      default:
        return 0x00;
    }
  }
  return 0x00; 
}