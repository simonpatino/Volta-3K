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
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(250E3);
  // Register the onReceive callback function
  LoRa.onReceive(onReceive);
  return 1;
}

void LoRaComm::transmitData(float dataList[], int length, byte dataID) {
  /*
    dataWrite type only works as long as all values are under 3276 and higher than -3276. Por eso revisamos que 
    todos los datos estén dentro de ese rango. Si no se cumple el rango, se utiliza la comunicación lenta de Strings.
    Cambiamos el dataID +=1 para que el receptor sepa que tipo de método de telecomunicación (write o print) se está usando

    La presición del método write es de solo una cifra decimal, si se quiere aumentar debe reducirse el rango válido y por ende
    tanto éste codigo como el código de recepción
  */
  if(LoRa.parsePacket()) return;
  bool outOfRangeFound = false;  // Flag to track if an out-of-range value is found
  for (int i = 0; i < length; i++) {
    if (dataList[i] > 3276.0 || dataList[i] < -3276.0) {
      outOfRangeFound = true;
      break;
    }
  }

  if(outOfRangeFound) {
    dataID += 1;
    String dataString = "";
    for (int i = 0; i < length; i++) {
      dataString += String(dataList[i], 1) + ",";
    }
    LoRa.beginPacket();
    LoRa.write(dataID);
    LoRa.print(",");
    LoRa.print(dataString);
  } else {
    LoRa.beginPacket();
    LoRa.write(dataID);
    //Este método envía todos los floats como bytes pero por lo mismo reduce la precisión, el rango y hace todo mas complejo.
    //Pro tip: do not change this unless you have time to ground test it very good.
    for (int i = 0; i < length; i++) {
      int16_t scaledData = (int16_t)(dataList[i] * 10); // Scale to preserve 1 decimal place
      LoRa.write((uint8_t*)&scaledData, sizeof(scaledData));
    }
  }
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

void LoRaComm::transmitGPS(int satNum, float lat, float lon) {
  /*
    Transmite los datos del GPS con 6 cifras después del punto decimal
  */
  String gpsData = String(satNum) + "," + String(lat, 6) + "," + String(lon, 6);
  Serial.println(gpsData);
  LoRa.beginPacket();
  LoRa.write(gpsDataID);  // Send the string with the values
  LoRa.print(gpsData);  // Send the string with the values
  LoRa.endPacket();
}

void LoRaComm::transmitPyroInfo(bool pyroContinuity[10]) {
  /*
    Transmite los datos bool de los pyro checks como bytes individuales
  */
  LoRa.beginPacket();
  LoRa.write(pyroDataID);
  for (int i = 0; i < 10; i++) {
    byte boolAsByte = pyroContinuity[i] ? 1 : 0;  // Convert bool to byte (1 or 0)
    LoRa.write(boolAsByte);  // Send each bool as a byte
  }
  LoRa.endPacket();
}

 