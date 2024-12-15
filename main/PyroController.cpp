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
      int pin = PYRO_FIRE_PINS_MATRIX[i][j];
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  }
  return 1;
}

/*
  Turns off all pyros
*/
void PyroController::killPyros() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      digitalWrite(PYRO_FIRE_PINS_MATRIX[i][j], LOW);
    }
  }
}

/*
  Fires the pyro when the ID (e.g 1A or 3B is inputed). An easier nomenclature is possible for sure
  but it's to late for that
  * selfContained makes this function both fire and kill the pyro with a constant delay
*/
bool PyroController::firePyro(int number, char letter, bool selfContained) { 
  int j = 0;
  if (letter == 'a' || letter == 'A') {
    j = 0;
  } else if(letter == 'b' || letter == 'b'){
    j = 1;
  } else {
    return 0;
  }
  int pyroPin = PYRO_FIRE_PINS_MATRIX[number - 1][j];
  digitalWrite(pyroPin, HIGH);
  if(selfContained) {
    delay(fireDelay);
    digitalWrite(pyroPin, LOW);
  }
  return 1;
}

void PyroController::checkContinuityAll(bool continuityPyros[]) {
  for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 2; j++) {
      continuityPyros[i+j] = digitalRead(PYRO_CHECKS[i][j]);
    }
  }
}

bool PyroController::checkContinuitySingle(int number, char letter) { 
  int j = 0;
  if (letter == 'a' || letter == 'A') {
    j = 0;
  } else if(letter == 'b' || letter == 'b'){
    j = 1;
  } else return 0;
  return digitalRead(PYRO_CHECKS[number - 1][j]);
}

void PyroController::readBayTempAll(float powderChambTemp[]) {
  for(int i = 0; i < 4; i++) {
    powderChambTemp[i] = analogRead(POWDERCHAMBERTEMP[i]);
  }
}

