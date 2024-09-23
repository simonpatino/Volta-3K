#ifndef CONSTANTS_H
#define CONSTANTS_H

#define LORA_FREQ 915E6
#define LORA_SYNC 0xAF
#define LORA_NSS 10
#define LORA_RST 9
#define LORA_DI0 29

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


#endif
