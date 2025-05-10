#include "LoRaComm.h"
#include <LoRa.h>
#include "Constants.h"

byte LoRaComm::lastCommand = 0;

LoRaComm::LoRaComm() {}

bool LoRaComm::begin() {
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa initialization failed!");
    return 0;
  }
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.setTxPower(15);  // Set TX power to minimum (2 dBm)
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(250E3);
  // Register the onReceive callback function
  LoRa.onReceive(onReceive);
  return 1;
}

void LoRaComm::transmitData(float dataList[], int length) {
  /*
    dataWrite type only works as long as all values are under 3276 and higher than -3276. Por eso revisamos que 
    todos los datos estén dentro de ese rango. Si no se cumple el rango, se utiliza la comunicación lenta de Strings.
    Cambiamos el dataID +=1 para que el receptor sepa que tipo de método de telecomunicación (write o print) se está usando

    La presición del método write es de solo una cifra decimal, si se quiere aumentar debe reducirse el rango válido y por ende
    tanto éste codigo como el código de recepción
  */
  if (LoRa.parsePacket()) return;
  String dataString = "";
  for (int i = 0; i < length -2 ; i++) {
    dataString += String(dataList[i], 1) + ",";
  }
  //The last two values are the GPS coordinates
  dataString += String(dataList[length - 2], 5) + ",";
  dataString += String(dataList[length - 1], 5);

  LoRa.beginPacket();
  LoRa.write(coreDataID);
  LoRa.print(dataString);
  LoRa.endPacket();
  LoRa.receive();
}

void LoRaComm::transmitConfirmation() {
  /*
    Transmite el mensaje de confirmación para parar el bucle de envío de comandos
  */
  LoRa.beginPacket();
  LoRa.write(confirmationCode);
  LoRa.endPacket();
}

void LoRaComm::transmitStageChange(int stage) {
  /*
    Transmite el mensaje de confirmación para parar el bucle de envío de comandos
  */
  LoRa.beginPacket();
  LoRa.write(stageDataID);
  LoRa.write(stage);
  LoRa.endPacket();
}

void LoRaComm::onReceive(int packetSize) {
  /*
    Callaback que recibe un mensaje de la estación terrena.
  */
  if (packetSize) {
    //El mensaje es específicamente el comando, lo guardamos para leerlo mas adelante
    LoRaComm::lastCommand = LoRa.read();
    //Envía mensaje de confirmación y espera un segundo para que no se interrumpa la comunicación siguiente a esta
    transmitConfirmation();
    delay(100);
  }
}

void LoRaComm::checkReceive() {
  //Si esta funcion no insiste en el .receive() el callback es mucho menos efectivo
  LoRa.receive();
}

void LoRaComm::transmitPyroInfo(bool pyroContinuity[10]) {
  /*
    Transmite los datos bool de los pyro checks como bytes individuales
  */
  LoRa.beginPacket();
  LoRa.write(pyroDataID);
  for (int i = 0; i < 10; i++) {
    byte boolAsByte = pyroContinuity[i] ? 1 : 0;  // Convert bool to byte (1 or 0)
    LoRa.write(boolAsByte);                       // Send each bool as a byte
  }
  LoRa.endPacket();
  checkReceive();
}

void LoRaComm::transmitString(String message) {
  /*
    Transmite los datos bool de los pyro checks como bytes individuales
  */
  LoRa.beginPacket();
  LoRa.write(stringID);
  LoRa.print(message);
  LoRa.endPacket();
  checkReceive();
}

void LoRaComm::setToSleep() {
  /*
    Manda el módulo a dormir
  */
  transmitString("I was put to sleep");
  LoRa.sleep();
}

void LoRaComm::wakeUp() {
  /*
    Manda el módulo a dormir
  */
  LoRa.idle();
}
