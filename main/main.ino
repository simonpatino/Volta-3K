#include "SensorConfig.h"

//Left Side
#define RX1 1
#define TX1 2
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

void setup() {
  
  Serial.begin(9600);

  bmeActivation();

  bnoActivation();

}

void loop() {
  // put your main code here, to run repeatedly:

}
