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

/*
  * dataID makes the difference between a general data packet or special data packet
    Special data may include: information that is relevant for a short period of time such as bay temperature, 
    a constant value such as the reference pressure
*/

void LoRaComm::transmitData(float dataList[], int dataID) {
    
    String dataString = "";
    for (int i = 0; i < 18; i++) {
        dataString += String(dataList[i], 2) + ",";
    }
    dataString += String(millis() / 1000);

    LoRa.beginPacket();
    LoRa.print(rocketAddress);
    LoRa.print(dataID);
    LoRa.print(dataString);
    LoRa.endPacket();
}
