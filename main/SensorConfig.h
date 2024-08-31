#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>                      
#include <LoRa.h>

//Left Side
#define RX1 0
#define TX1 1
#define CS_FLASH 2
#define CS02 4
#define CS03 5
#define B 6
#define G 7
#define R 8
#define RST 9
#define CS_LORA 10
#define MOSI0 11
#define MISO0 12
#define SCL 24
#define SDA 25
#define AOUT1 26
#define BOUT1 27
#define INT_LORA 29
#define PYRO5_A 30
#define P_CHK5B 31
#define PYRO5_B 32

//Right Side
#define BOUT2 23
#define AOUT2 22
#define P_CHK1A 21
#define PYRO1_A 20
#define P_CHK1B 19
#define PYRO1_B 18
#define P_CHK2A 17
#define PYRO2_A 16
#define P_CHK2B 15
#define PYRO2_B 14
#define SCK0 13
#define P_CHK3A 41
#define PYRO3_A 40
#define P_CHK3B 39
#define PYRO3_B 38
#define P_CHK4A 37
#define PYRO4_A 36
#define P_CHK4B 35
#define PYRO4_B 34
#define P_CHK5A 33

/* ----- LORA SETTINGS ----- */
#define LORA_FREQ 915E6
#define LORA_SYNC 0xAF
#define LORA_NSS 10
#define LORA_RST 9
#define LORA_DI0 29

const int PYRO_FIRE[5][2] = {
  { PYRO1_A, PYRO1_B },
  { PYRO2_A, PYRO2_B },
  { PYRO3_A, PYRO3_B },
  { PYRO4_A, PYRO4_B },
  { PYRO5_A, PYRO5_B }
};
const int PYRO_CHECKS[5][2] = {
  { P_CHK1A, P_CHK1B },
  { P_CHK2A, P_CHK2B },
  { P_CHK3A, P_CHK3B },
  { P_CHK4A, P_CHK4B },
  { P_CHK5A, P_CHK5B }
};

//Function to eject pyro

void checkActivation() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(PYRO_CHECKS[i][j], INPUT);
      pinMode(PYRO_FIRE[i][j], OUTPUT);
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  }
}

void killPyros() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  }
}

bool firePyro(int id_num, char id_letter) {
  int j;
  if (id_letter == 'a' || id_letter == 'A') {
    j = 0;
  } else if (id_letter == 'b' || id_letter == 'B') {
    j = 1;
  } else {
    return 0;
  }
  Serial.println(PYRO_FIRE[id_num][j]);
  digitalWrite(PYRO_FIRE[id_num][j], HIGH);
  return 1;
}

void ejectEvent(int pinToEject) {

  digitalWrite(pinToEject, HIGH);

  String message = "The pin" + String(pinToEject) + " was ACTIVATED";

  Serial.println(message);
}

//Function to check if pyro has continuity

int pyroCheck(int pyroChannel) {

  int state = digitalRead(pyroChannel);

  if (state) {

    //String message = "The channel  " + String(pyroChannel) + " is ON";
    String message = "ON";

    Serial.print(message);

    return 1;

  } else {


    //String message = "The channel  " + String(pyroChannel) + " is OFF";
    String message = "OFF";

    Serial.print(message);

    return 0;
  }
}

//BME280 configuration (I2C)

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;  // I2C

void bmeActivation() {

  unsigned status;

  status = bme.begin(0x77, &Wire2);

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  Serial.println("BME280 Found!");
}


//bno055

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

void bnoActivation() {

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  } else {

    Serial.println("BNO055 Found!");
  }
}




//XTSD 512MB (SPI)

File dataFile;

void xtsdActivation() {

  SPI.setMOSI(11);  // Audio shield has MOSI on pin 7
  SPI.setMISO(12);  // Audio shield has MOSI on pin 7
  SPI.setSCK(13);   // Audio shield has SCK on pin 14

  if (!SD.begin(CS_FLASH)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD Found!");
}

//GPS

SFE_UBLOX_GPS myGPS;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void gpsActivation() {

  Wire2.begin();

  if (myGPS.begin(Wire2) == false) {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }
}

void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}


//LoRa RFM95W

//#define RF95_FREQ 915.0

//RH_RF95 rf95(CS_LORA, INT_LORA);

void loraActivation() {
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed");
  }

  LoRa.setSyncWord(LORA_SYNC);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(250E3);
  // LoRa.setTxPower(17);
}

void transmitLoRa(String message) {
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}
//GPS

//static const uint32_t GPsBaud = 4800;

//TinyGPSPlus gps;


//void gpsActivation() {

//Serial1.begin(GPsBaud);

//}
