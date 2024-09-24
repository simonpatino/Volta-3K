#include "LoRaComm.h"
#include <LoRa.h>
#include "Constants.h"

LoRaComm::LoRaComm() {}

bool LoRaComm::begin() {
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa initialization failed!");
    return 0;
  }
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(250E3);
  return 1;
}

void LoRaComm::transmitData(float bmeVariables[], float bnoVariables[],
                  bool pyroVariables[], long gpsVariables[], 
                  float message[]){

    message[0] = bmeVariables[0];
    message[1] = bmeVariables[1];
    message[2] = bmeVariables[2];
    message[3] = bmeVariables[3];
    message[4] = bnoVariables[0];
    message[5] = bnoVariables[1];
    message[6] = bnoVariables[2];
    message[7] = pyroVariables[0];
    message[8] = pyroVariables[1];
    message[9] = pyroVariables[3];
    message[10] = pyroVariables[4];
    message[11] = pyroVariables[5];
    message[12] = pyroVariables[6];
    message[13] = pyroVariables[7];
    message[14] = pyroVariables[8];
    message[15] =  gpsVariables[0];
    message[16] =  gpsVariables[1];
    message[17] = millis()/1000; 

    //This message[] variable is the same that use logData

    String data = "";
    for (int i = 0; i < 18; i++) {
        data += String(message[i], 2) + ",";
    }
    data += String(millis() / 1000);

    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
}
