#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <Adafruit_BNO08x.h>
//#include <SD.h>
//#include <RH_RF95.h>
//#include <TinyGPSPlus.h>
//#include <SoftwareSerial.h>

//Left Side
#define RX1 0
#define TX1 1
#define CS_FLASH 3
#define INT_LORA 4
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
#define CS02 28
#define CS03 29
#define P_CHK5A 30
#define PYRO5_B 31
#define P_CHK5B 32

//Right Side
#define BOUT2 23
#define AOUT2 22
#define PYRO1_A 21
#define P_CHK1A 20
#define PYRO1_B 19
#define P_CHK1B 18
#define PYRO2_A 17
#define P_CHK2A 16
#define PYRO2_B 15
#define P_CHK2B 14
#define PYRO3_A 13
#define SCK0 41
#define P_CHK3A 40
#define PYRO3_B 39
#define P_CHK3B 38
#define PYRO4_A 37
#define P_CHK4A 36
#define PYRO4_B 35
#define P_CHK4B 34
#define PYRO5_A 33

//Function to eject pyro

void ejectEvent(int pinToEject) {


  digitalWrite(pinToEject, HIGH);

  String message = "The pin" + String(pinToEject) + " was ACTIVATED";

  Serial.println(message);
}

//Function to check if pyro has continuity

int pyroCheck(int pyroChannel) {


  int state = digitalRead(pyroChannel);

  if (state) {

    String message = "The channel  " + String(pyroChannel) + " is ON";

    Serial.println(message);

    return 1;

  } else {


    String message = "The channel  " + String(pyroChannel) + " is OFF";

    Serial.println(message);

    return 0;
  }
}

//BME280 configuration (I2C)

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;  // I2C

void bmeActivation() {

  unsigned status;

  status = bme.begin();

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

//XTSD 512MB (SPI)

//Sd2Card card;
//SdVolume volume;
//SdFile root;

//void xtsdActivation() {

  //pinMode(CS_FLASH, OUTPUT);

  //if (!card.init(SPI_HALF_SPEED, CS_FLASH)) {
    //Serial.println("SD initialization failed. Things to check:");
    //Serial.println("* is a card inserted?");
    //Serial.println("* is your wiring correct?");
    //Serial.println("* did you change the chipSelect pin to match your shield or module?");
    //return;
  //} else {
   // Serial.println("Wiring is correct and a card is present.");
  //}

  //Serial.print("\nCard type: ");
  //switch (card.type()) {
    //case SD_CARD_TYPE_SD1:
      //Serial.println("SD1");
      //break;
    //case SD_CARD_TYPE_SD2:
      //Serial.println("SD2");
      //break;
    //ase SD_CARD_TYPE_SDHC:
      //Serial.println("SDHC");
      //break;
    //default:
      //Serial.println("Unknown");
  //}

  //if (!volume.init(card)) {
    //Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    //return;
  //}

  //uint32_t volumesize;
  //Serial.print("\nVolume type is FAT");
  //Serial.println(volume.fatType(), DEC);
  //Serial.println();

  //volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
  //volumesize *= volume.clusterCount();     // we'll have a lot of clusters
  //if (volumesize < 8388608ul) {
    //Serial.print("Volume size (bytes): ");
    //Serial.println(volumesize * 512);  // SD card blocks are always 512 bytes
  //}
  //Serial.print("Volume size (Kbytes): ");
  //volumesize /= 2;
  //Serial.println(volumesize);
  //Serial.print("Volume size (Mbytes): ");
  //volumesize /= 1024;
  //Serial.println(volumesize);
//}

//LoRa RFM95W

//#define RF95_FREQ 915.0

//RH_RF95 rf95(CS_LORA, INT_LORA);

//void loraActivation() {

  //pinMode(RST, OUTPUT);

  //digitalWrite(RST, HIGH);

  //manual reset
  //digitalWrite(RST, LOW);
  //delay(10);
  //digitalWrite(RST, HIGH);
  //delay(10);

  //while (!rf95.init()) {
    //Serial.println("LoRa radio init failed");
    //while (1)
      //;
  //}
  //erial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  //if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    //while (1)
    //  ;
  //}
  //Serial.print("Set Freq to: ");
  //Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(23, true);
//}

//GPS

//static const uint32_t GPsBaud = 4800;

//TinyGPSPlus gps;


//void gpsActivation() {

  //Serial1.begin(GPsBaud);

//}
