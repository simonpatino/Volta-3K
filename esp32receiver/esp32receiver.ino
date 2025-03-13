#include <SPI.h>
#include <LoRa.h>

// LoRa configuration
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26

byte onAwait = 0x00;
int insistDelay = 200;
long lastTransmit = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("LoRa Receiver Station");

  // Initialize LoRa pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  // Start LoRa communication
  if (!LoRa.begin(915E6)) {  // Set frequency (915 MHz for North America, change if needed)
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setSyncWord(0xAF);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(250E3);

  Serial.println("LoRa Initialized");
}


/*
  * dataID makes the difference between a general data packet or special data packet
    Special data may include: information that is relevant for a short period of time such as bay temperature, 
    a constant value such as the reference pressure
*/
void loop() {
  // Check for incoming packets
  if (LoRa.parsePacket()) {
    byte dataID;           // Use byte instead of int for dataID
    dataID = LoRa.read();  // Read the single byte for dataID
    Serial.print(dataID);
    Serial.print(": ");
    if (dataID == 0x00 || dataID == 0x03) {
      //Mensaje CORE en modo simple o mensaje de GPS
      while (LoRa.available()) {
        String received = LoRa.readString();
        Serial.print(received);
      }
    } else if(dataID == 0x02) {
      while (LoRa.available()) {
        String received = LoRa.readString();
        Serial.print(received);
      }
    } else if (dataID == 0x07) {
      //Mensaje Confirmación de que sí se recibió el cohete.
      onAwait = 0;
      Serial.print("Confirmation Received");
    } else if (dataID == 0x04) {
      //Mensaje con bools de los canales pirotécnicos
      bool pyroContinuity[10];  // Array to store the received bool values

      // Read the 10 bool values, each as a separate byte
      for (int i = 0; i < 10; i++) {
        byte receivedByte = LoRa.read();          // Read the byte (0 or 1)
        pyroContinuity[i] = (receivedByte == 1);  // Convert byte to bool (1 -> true, 0 -> false)
      }

      // Optionally, print the received bool values for debugging
      for (int i = 0; i < 10; i++) {
        Serial.print(pyroContinuity[i]);
        Serial.print(" ");
      }
    } else if (dataID == 0x06) {

    } else {
      Serial.print("I received trash");
    }
    Serial.println();
  }

  if (onAwait != 0x00) {
    /*
      Si estamos en proceso de enviar un comando y no hemos recibido el acknowledgment entonces
      reenviamos el mensaje infinitamente cada insistDelay (ms) antes de volver a enviar el mensaje
    */
    if (int(millis() - lastTransmit) > insistDelay) {
      LoRa.beginPacket();
      LoRa.write(onAwait);
      LoRa.endPacket();
      Serial.println("Command sent");
      lastTransmit = millis();
    }
  } else {
    /*
      Si no estamos en proceso de enviar un comando, revisamos si el operador a indicado un comando para enviar
    */
    onAwait = getCommand();
  }
}

byte getCommand() {
  if (Serial.available()) {
    char incomingChar = Serial.read();  // Read the incoming character
    if (incomingChar == 's') {
      //Comando para dar confirmación de que hay ground station
      return 0x01;
    } else if (incomingChar == 'p') {
      //Comando para recibir info Pyros
      return 0x04;
    } else if (incomingChar == 'c') {
      //Comando para recibir info ejection chambers
      return 0x05;
    } else if (incomingChar == 'v') {
      //Comando para activar el sistema de cámaras
      return 0x06;
    }
    else {
      //No estamos en proceso de enviar un comando en este momento
      return 0x00;
    }
  }
}