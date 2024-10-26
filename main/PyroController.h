#ifndef PYRO_H
#define PYRO_H
#include "Constants.h"

class PyroController {
  public:
    PyroController();
    bool begin();
    void checkContinuityAll(bool continuityPyros[]);  //Check All Pins
    void readBayTempAll(float dataArray[]);
    bool checkContinuitySingle(int number, char letter);
    void killPyros();
    void firePyro(int number, char letter);
    void ejectEvent(int pinToEject);
    void pyroCheck(int pyroChannel); //Check an Individual Pin
  private: 
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
};


#endif