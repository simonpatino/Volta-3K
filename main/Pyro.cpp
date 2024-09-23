#include "Pyro.h"
#include "Constants.h"
#include "save.h"

void setupPyro() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(PYRO_CHECKS[i][j], INPUT);
      pinMode(PYRO_FIRE[i][j], OUTPUT);
      digitalWrite(PYRO_FIRE[i][j], LOW);
    }
  }
}


void checkPyro() {
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 2; j++) {
            int pyroState = digitalRead(PYRO_CHECKS[i][j]);
            Serial.print("Pyro ");
            Serial.print(i);
            Serial.print(j);
            Serial.print(": ");
            Serial.println(pyroState ? "ON" : "OFF");
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
