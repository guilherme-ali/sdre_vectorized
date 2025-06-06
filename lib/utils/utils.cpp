#include "utils.h"
#include <AutoLQR.h>

#define STATE_SIZE 6
#define CONTROL_SIZE 3

// Declaração externa do controlador (definido na main.cpp)
extern AutoLQR controller;

void displayGains(){
    float exportedGains[CONTROL_SIZE * STATE_SIZE];
    controller.exportGains(exportedGains);

    float exportedKr[CONTROL_SIZE * CONTROL_SIZE];
    controller.exportKr(exportedKr);
    Serial.println("Ganhos do LQR calculados com sucesso");

    // Exporta os ganhos calculados
    Serial.println("Ganhos Exportados (K):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            Serial.print(exportedGains[i * STATE_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Exporta e imprime a matriz Kr
    Serial.println("Matriz Kr (ganho de referência):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Serial.print(exportedKr[i * CONTROL_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void displayIMU() {
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);
    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(), 6);
}

void displayStates(float states[]) { 
    Serial.print("Roll:"); Serial.print(states[0]); // Roll em radianos
    Serial.print(",Pitch:"); Serial.print(states[1]); // Pitch em radianos
    Serial.print(",Yaw:"); Serial.print(states[2]); // Yaw em radianos
    Serial.print(",p:"); Serial.print(states[3]);
    Serial.print(",q:"); Serial.print(states[4]);
    Serial.print(",r:"); Serial.println(states[5]);
}

void displayControlSignals(float u_signal[], float thrust_signal) {
    Serial.print("u1:"); Serial.print(u_signal[0]);
    Serial.print(",u2:"); Serial.print(u_signal[1]);
    Serial.print(",u3:"); Serial.print(u_signal[2]);
    Serial.print(",T:"); Serial.println(thrust_signal);
}

void displayMotorOmegaSq(float thrust_signal, float u_torques[], float b_coeff, float d_coeff) {
    if (b_coeff == 0.0f || d_coeff == 0.0f) {
        Serial.println("Erro: Coeficientes b ou d não podem ser zero em displayMotorOmegaSq.");
        return;
    }

    float inv_4b = 1.0f / (4.0f * b_coeff);
    float inv_2b = 1.0f / (2.0f * b_coeff);
    float inv_4d = 1.0f / (4.0f * d_coeff);

    float u1 = thrust_signal;    // Empuxo total
    float u2 = u_torques[0];     // Torque de Rolagem
    float u3 = u_torques[1];     // Torque de Arfagem
    float u4 = u_torques[2];     // Torque de Guinada

    float w1_sq = u1 * inv_4b           - u3 * inv_2b - u4 * inv_4d;
    float w2_sq = u1 * inv_4b - u2 * inv_2b           + u4 * inv_4d;
    float w3_sq = u1 * inv_4b           + u3 * inv_2b - u4 * inv_4d;
    float w4_sq = u1 * inv_4b + u2 * inv_2b           + u4 * inv_4d;

    Serial.print("w1_sq: "); Serial.print(w1_sq);
    Serial.print(", w2_sq: "); Serial.print(w2_sq);
    Serial.print(", w3_sq: "); Serial.print(w3_sq);
    Serial.print(", w4_sq: "); Serial.println(w4_sq);
}