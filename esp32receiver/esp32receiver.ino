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

// Command code for frequency change
#define CMD_SET_FREQUENCY 0x08
// Special code to indicate waiting for frequency input via Serial
#define AWAIT_FREQUENCY_INPUT 0xFF


// onAwait: Tracks if we are waiting for operator command from terminal
//          0x00 means waiting for command
//          0xFF means waiting for frequency input after 'l' command
//          Other values indicate specific command states (e.g., 0x08 for frequency change)
byte onAwait = 0x00;

// Delay between retransmission attempts in milliseconds
int insistDelay = 200;

// Timestamp of last transmission, used for timing retries
long lastTransmit = 0;

// Variable to store the frequency string to be sent
String frequencyStringToSend = "";

void setup() {
  // Initialize Serial communication at 9600 baud rate
  Serial.begin(115200);
  // Wait for Serial connection to be established
  while (!Serial)
    ;

  Serial.println("LoRa Receiver Station");

  // Configure LoRa module with defined pins for SPI communication
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  // Initialize LoRa at 915MHz frequency (North American band)
  // If initialization fails, print error and halt program
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

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
  // Check for incoming packets
  if (LoRa.parsePacket()) {
    // Each packet starts with a single byte ID indicating the message type
    byte dataID;           
    dataID = LoRa.read();  
    Serial.print(dataID);
    Serial.print(": ");


    /*
     * Ground Station Message Protocol Documentation
     * 
     * The ground station receives messages from the rocket with the following ID format:
     * 
     * 0x00: Core Telemetry Data
     * - Contains main flight parameters as comma-separated string
     * - Format Example: "Iteration,Time,Accel[3],Alt,Press,Euler[3],MaxAlt,Stage,Sat#,Lat,Long"
     * - Example: "156,12.5,-0.5,9.8,1.2,1050,101.3,0,90,180,1200,2,8,51.5,-0.1"
     * 
     * 0x02: Secondary Telemetry Data
     * - Dafault ID send for the rocket when it turns on
     * - Format: "Gps Data"
     * - Example: "Gps Data"
     * 
     * 0x03: GPS Position Data
     * - Contains GPS coordinates and related info as string
     * - Format similar to core telemetry but focused on location data
     * 
     * 0x04: Pyro Channel Status
     * - Contains 10 boolean values (0/1) for pyro channel continuity
     * - Each channel status sent as single byte
     * - Example: 1 1 0 1 0 0 0 0 1 1 (channels 1,2,4,9,10 have continuity)
     * 
     * 0x06: Reserved
     * - Reserved for future protocol expansion
     * 
     * 0x07: Command Acknowledgment
     * - Confirmation that rocket received ground command
     * - Contains no additional data
     */

    // Handle different message types based on dataID
    if (dataID == 0x00 || dataID == 0x03) {
      // 0x00: Core telemetry data in simple format
      // 0x03: GPS position data
      // Both message types contain string data that should be printed directly
      while (LoRa.available()) {
        String received = LoRa.readString();
        Serial.print(received);
        Serial.print(LoRa.rssi());
      }
    } else if(dataID == 0x02) {
      // 0x02: Secondary telemetry data
      // Contains string data that should be printed directly
      while (LoRa.available()) {
        String received = LoRa.readString();
        Serial.print(received);
        
      }
    } else if (dataID == 0x07) {
      // 0x07: Acknowledgment message from rocket
      // Indicates rocket received our last command
      Serial.print("Confirmation Received for command 0x");
      Serial.println(onAwait, HEX); // Print the command that was acknowledged

      // Check if the acknowledged command was the frequency change command
      if (onAwait == CMD_SET_FREQUENCY && frequencyStringToSend.length() > 0) {
        // Attempt to parse the frequency string to a long integer using atol for better range
        long newFrequencyHz = atol(frequencyStringToSend.c_str()); 
        
        // Validate if the parsed frequency is within the allowed range (905 MHz to 930 MHz)
        if (newFrequencyHz >= 905E6 && newFrequencyHz <= 930E6) { 
            Serial.print("Attempting to change local frequency to: ");
            Serial.print(frequencyStringToSend);
            Serial.println(" Hz");
            // Call setFrequency directly without checking the return value
            LoRa.setFrequency(newFrequencyHz);
            // Assume the change was initiated, cannot confirm success via return value
            Serial.println("Local LoRa frequency change initiated.");
        } else {
            Serial.print("Error: Requested frequency ");
            Serial.print(frequencyStringToSend);
            Serial.println(" Hz is outside the allowed range (905-930 MHz).");
            Serial.println("Local frequency NOT changed.");
        }
      }

      //comment 

      onAwait = 0x00; // Clear waiting state regardless of which command was acknowledged
      frequencyStringToSend = ""; // Clear frequency string storage
    } else if (dataID == 0x04) {
      // 0x04: Pyro channel continuity status
      // Contains 10 boolean values indicating continuity of each pyro channel
      bool pyroContinuity[10];  

      // Each continuity value is sent as a single byte
      // 0x01 = true (continuity)
      // 0x00 = false (no continuity)
      for (int i = 0; i < 10; i++) {
        byte receivedByte = LoRa.read();          
        pyroContinuity[i] = (receivedByte == 1);  
      }

      // Print continuity status for all channels
      // 1 indicates continuity, 0 indicates no continuity
      for (int i = 0; i < 10; i++) {
        Serial.print(pyroContinuity[i]);
        Serial.print(" ");
      }
    } else if (dataID == 0x06) {
      // 0x06: Reserved for future use
      // Currently no implementation
    } else {
      // Unknown message type received
      Serial.print("I received trash");
    }
    Serial.println(); // End the line after processing any message type
  }

  if (onAwait == AWAIT_FREQUENCY_INPUT) {
    // We are waiting for the user to type the frequency in MHz via Serial
    if (Serial.available() > 0) {
      String freqStrMHz = Serial.readStringUntil('\n');
      freqStrMHz.trim(); // Remove potential whitespace/newlines

      // Validate the frequency string *before* sending anything
      if (freqStrMHz.length() > 0) {
        // Parse the input string as a float (MHz)
        float requestedFrequencyMHz = atof(freqStrMHz.c_str()); 
        // Convert MHz to Hz
        long requestedFrequencyHz = (long)(requestedFrequencyMHz * 1E6);

        // Check if the calculated Hz frequency is within the valid range
        if (requestedFrequencyHz >= 905E6 && requestedFrequencyHz <= 930E6) {
          // Frequency is valid, proceed to send the command
          // Store the frequency *in Hz* as a string for sending
          frequencyStringToSend = String(requestedFrequencyHz); 
          Serial.print("Frequency ");
          Serial.print(freqStrMHz);
          Serial.print(" MHz (");
          Serial.print(frequencyStringToSend);
          Serial.println(" Hz) is valid. Attempting to send frequency change command.");
          
          onAwait = CMD_SET_FREQUENCY; // Set state to send the frequency command

          // Send the first packet immediately
          LoRa.beginPacket();
          LoRa.write(onAwait); // Write command code 0x08
          LoRa.print(frequencyStringToSend); // Write frequency *in Hz* as string
          LoRa.endPacket();
          Serial.println("Command sent.");
          lastTransmit = millis(); // Start timer for potential retransmissions
        } else {
          // Frequency is outside the allowed range
          Serial.print("Error: Requested frequency ");
          Serial.print(freqStrMHz);
          Serial.println(" MHz is outside the allowed range (905-930 MHz).");
          Serial.println("Command NOT sent. Please try again.");
          onAwait = 0x00; // Reset state, wait for new command
          frequencyStringToSend = ""; // Clear stored frequency
        }
      } else {
        // Input string was empty or invalid for parsing
        Serial.println("Invalid frequency entered. Please enter frequency in MHz (e.g., 915 or 920.5).");
        onAwait = 0x00; // Reset state, wait for new command
        frequencyStringToSend = ""; // Clear stored frequency
      }
    }
  } else if (onAwait != 0x00) {
    // If we are in the process of sending a command (which is now guaranteed to be valid) 
    // and haven't received acknowledgment, continuously retry sending the command 
    // every insistDelay milliseconds until acknowledged
    if (int(millis() - lastTransmit) > insistDelay) {
      // Begin LoRa packet transmission
      LoRa.beginPacket();
      // Write the command byte
      LoRa.write(onAwait);
      // If the command is to set frequency, also send the frequency string
      if (onAwait == CMD_SET_FREQUENCY) {
          LoRa.print(frequencyStringToSend);
      }
      // End and send the packet
      LoRa.endPacket();
      Serial.print("Command sent: 0x"); Serial.println(onAwait, HEX);
      // Update timestamp of last transmission
      lastTransmit = millis();
    }
  } else {
    // If we're not currently sending a command or waiting for frequency input,
    // check if operator has input a new command to send
    onAwait = getCommand();
    // If a command other than 'wait for frequency' was entered, send it immediately
    if (onAwait != 0x00 && onAwait != AWAIT_FREQUENCY_INPUT) {
        LoRa.beginPacket();
        LoRa.write(onAwait);
        // Note: No extra data needed here for commands other than 0x08
        LoRa.endPacket();
        Serial.print("Command sent: 0x"); Serial.println(onAwait, HEX);
        lastTransmit = millis(); // Start timer for potential retransmissions
    }
  }
}

/**
 * Gets a command from the operator via Serial input and returns the corresponding command code.
 * 
 * Command codes:
 * 0x01 - 's' - Send ground station confirmation | Allow enter the rocket to the next stage
 * 0x04 - 'p' - Request pyro channel status | Check if the pyro channels are working
 * 0x05 - 'c' - Request ejection chamber info | Check if the ejection chambers are working
 * 0x06 - 'o' - Activate camera system 
 * 0x07 - 'f' - Deactivate camera system
 * 0x08 - 'l' - Change LoRa frequency (followed by frequency input prompt)
 * 0xFF - Internal code: Waiting for frequency input after 'l'
 * 0x00 - Any other character or no input
 * 
 * @return byte Command code based on Serial input character, or 0xFF if waiting for frequency.
 */
byte getCommand() {
  if (Serial.available()) {
    char incomingChar = Serial.read();  // Read the incoming character
    
    switch(incomingChar) {
      case 's':
        // Send confirmation that ground station is active
        return 0x01;
        
      case 'p': 
        // Request status of pyro channels (continuity)
        return 0x04;
        
      case 'c':
        // Request temperature/status of ejection chambers
        return 0x05;
        
      case 'o':
        // Activate onboard camera recording system
        return 0x06;

      case 'f':
        // Deactivate onboard camera recording system
        return 0x07;

      case 'l':
        // Initiate LoRa frequency change
        Serial.print("Enter new LoRa frequency in MHz (e.g., 915 or 920.5): "); // Updated prompt
        return AWAIT_FREQUENCY_INPUT; // Return special code to wait for frequency input in loop()
        
      default:
        // No valid command, return idle state
        return 0x00;
    }
  }
  return 0x00; // Return idle state if no Serial input available
}