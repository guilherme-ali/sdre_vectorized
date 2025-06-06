#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "MPU9250.h"

// Declarações das funções auxiliares
void displayGains();
void displayIMU();
void displayStates(float states[]); 
void displayControlSignals(float u_signal[], float thrust_signal);
void displayMotorOmegaSq(float thrust_signal, float u_torques[], float b_coeff, float d_coeff);

// Variáveis globais necessárias para as funções auxiliares
extern MPU9250 IMU;

#endif