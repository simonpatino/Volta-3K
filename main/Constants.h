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
#define CS_SD BUILTIN_SDCARD
#define CS02 4
#define CS03 5
#define BLED 6
#define GLED 7
#define RLED 8
#define RST 9
#define CS_LORA 10
#define MOSI0 11
#define MISO0 12
#define SCL 24
#define SDA 25
#define AOUT1 26
#define BOUT1 27
#define INT_LORA 29
#define FLASH_SPI_MOSI 11
#define FLASH_SPI_MISO 12
#define FLASH_SPI_SCK 13



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
#define PYRO5_A 30
#define P_CHK5B 31
#define PYRO5_B 32

const int POWDERCHAMBERTEMP_PINS[] = {26, 27, 28, 22};

#define IMU_ADDRESS 0x28

/*
  * God knows what i meant when i called them "ODR"
*/
enum ODR_MODES {
  LOW_RATE = 0,
  MID_RATE = 1,
  HIGH_RATE = 2,

  /* IMU ONLY PREDEFINED FUSION MODES
      * Both work at 100Hz
  */
  IMUONLY_IMU = 0x03, //The datasheed called it "IMU mode", don't blame me   | MAX ACC = +/- 4g
  IMUONLY_NDOF = 0x04, //Does anyone know what the fuck does NDOF stand for? | MAX ACC = +/- 4g

};

enum ACC_RANGES {
  ACC_RNG_2G = (0X00),
  ACC_RNG_4G = (0X01),
  ACC_RNG_8G = (0X02),
  ACC_RNG_16G = (0X03)
};

enum ACC_BW {
  ACC_BW_7_81HZ  =  (0x00),
  ACC_BW_15_63HZ =  (0x01),
  ACC_BW_31_25HZ =  (0x02),
  ACC_BW_62_5HZ  =  (0X03),
  ACC_BW_125HZ   =  (0X04),
  ACC_BW_250HZ   =  (0X05),
  ACC_BW_500HZ   =  (0X06),
  ACC_BW_1000HZ  =  (0X07)
};

enum ACC_PW {
  ACC_NORMAL      =  (0X00),
  ACC_SUSPEND     =  (0X01),
  ACC_LOWPOWER_1  =  (0X02),
  ACC_STANDBY     =  (0X03),
  ACC_LOWPOWER_2  =  (0X04),
  ACC_DEEPSUSPEND =  (0X05)
};

enum STAGES {
  STARTUP = (0x00),
  IDLE      =  (0X01),
  BOOSTING     =  (0X02),
  COASTING  =  (0X03),
  DROGUEDESCENT     =  (0X04),
  MAINDESCENT  =  (0X05),
  TOUCHDOWN =  (0X06)
};

#endif
