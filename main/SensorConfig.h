#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
//#include <RH_RF95.h>
//#include <TinyGPSPlus.h>
//#include <SoftwareSerial.h>

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

const int PYRO_FIRE[5][2] = {
  {PYRO1_A, PYRO1_B},
  {PYRO2_A, PYRO2_B},
  {PYRO3_A, PYRO3_B},
  {PYRO4_A, PYRO4_B},
  {PYRO5_A, PYRO5_B}
};
const int PYRO_CHECKS[5][2] = {
  {P_CHK1A, P_CHK1B},
  {P_CHK2A, P_CHK2B},
  {P_CHK3A, P_CHK3B},
  {P_CHK4A, P_CHK4B},
  {P_CHK5A, P_CHK5B}
};

//Function to eject pyro

void  checkActivation() {
  for(int i = 0 ; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(PYRO_CHECKS[i][j], INPUT);
      pinMode(PYRO_FIRE[i][j], OUTPUT);
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  } 
}

void killPyros() {
  for(int i = 0 ; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  } 
}

bool firePyro(int id_num, char id_letter) {
  int j;
  if(id_letter == 'a' || id_letter == 'A') {
    j = 0;
  } else if(id_letter == 'b' || id_letter == 'B') {
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

   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } else {

    Serial.println("BNO055 Found!");

  }
  
}




//XTSD 512MB (SPI)

File dataFile;

void xtsdActivation() {

  SPI.setMOSI(11);  // Audio shield has MOSI on pin 7
  SPI.setMISO(12);  // Audio shield has MOSI on pin 7
  SPI.setSCK(13);  // Audio shield has SCK on pin 14

  if (!SD.begin(CS_FLASH)) {
    Serial.println("SD initialization failed!");
    return;
  }
    Serial.println("SD Found!");

}
  
  
  
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
