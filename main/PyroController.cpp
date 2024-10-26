#include <Arduino.h>
#include "PyroController.h"
#include "Constants.h"

PyroController::PyroController() {}

bool PyroController::begin() {
  for (int i = 0; i < 4; i++) {
    pinMode(POWDERCHAMBERTEMP[i], INPUT);
  }
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(PYRO_CHECKS[i][j], INPUT);
      pinMode(PYRO_FIRE[i][j], OUTPUT);
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  }
  return 1;
}

void PyroController::killPyros() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  }
}

/*
  Fires the pyro when the ID (e.g 1A or 3B is inputed). An easier nomenclature is possible for sure
  but it's to late for that
*/
void PyroController::firePyro(int number, char letter) { 
  int j;
  if (letter == 'a' || letter == 'A') {
    j = 0;
  } else if(letter == 'b' || letter == 'b'){
    j = 1;
  }
  digitalWrite(PYRO_FIRE[number - 1][j], LOW);
}

void PyroController::checkContinuityAll(bool continuityPyros[]) {
  for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 2; j++) {
      continuityPyros[i+j] = digitalRead(PYRO_CHECKS[i][j]);
    }
  }
}

bool PyroController::checkContinuitySingle(int number, char letter) { 
  int j;
  if (letter == 'a' || letter == 'A') {
    j = 0;
  } else if(letter == 'b' || letter == 'b'){
    j = 1;
  }
  return digitalRead(PYRO_FIRE[number - 1][j]);
}

void PyroController::readBayTempAll(float powderChambTemp[]) {
  for(int i = 0; i < 4; i++) {
    powderChambTemp[i] = analogRead(POWDERCHAMBERTEMP[i]);
  }
}

