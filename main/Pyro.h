#ifndef PYRO_H
#define PYRO_H

void setupPyro();
void checkPyro();  //Check All Pins
void killPyros();
bool firePyro(int id_num, char id_letter);
void ejectEvent(int pinToEject);
void pyroCheck(int pyroChannel); //Check an Individual Pin

#endif