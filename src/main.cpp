#include <AutoLQR.h>

#define STATE_SIZE 6
#define CONTROL_SIZE 3

const float Ixx = 0.00184;  // Momento de inércia em torno do eixo x
const float Iyy = 0.00225;  // Momento de inércia em torno do eixo y   
const float Izz = 0.00338;  // Momento de inércia em torno do eixo z
const float Ir = 0.0001;   // Momento de inércia do conjunto do motor e hélice
const float m = 1.0;   // Massa do corpo rígido
const float g = 9.81;  // Aceleração gravitacional
const float l = 0.5;   // Distância do centro de massa até o ponto de aplicação da força
const float p = 62;   // velocidade angular em torno do eixo x
const float q = 62;   // velocidade angular em torno do eixo y
const float r = 62;   // velocidade angular em torno do eixo z
const float omega_r = 1000; // velocidade angular do motor

// System matrices
float A[STATE_SIZE * STATE_SIZE] = {
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, ((Iyy-Izz)/(2*Ixx))*r - Ir*omega_r/Ixx, ((Iyy-Izz)/(2*Ixx))*q,
    0, 0, 0, ((Izz-Ixx)/(2*Iyy))*r - Ir*omega_r/Iyy, 0, ((Izz-Ixx)/(2*Iyy))*p,
    0, 0, 0, ((Ixx-Iyy)/(2*Izz))*q, ((Ixx-Iyy)/(2*Izz))*p, 0
};

float B[STATE_SIZE * CONTROL_SIZE] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    1/Ixx, 0, 0,
    0, 1/Iyy, 0,
    0, 0, 1/Izz
};

// Cost matrices for normal operation
float Q[STATE_SIZE * STATE_SIZE] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

// Controller and state variables
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

void setup()
{
    Serial.begin(115200);

    // Initialize controller with system dynamics
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);

    // Compute optimal gains
    unsigned long startTime = micros(); // Captura o tempo inicial
    
    if (controller.computeGains()) {
        unsigned long endTime = micros(); // Captura o tempo final
        unsigned long executionTime = endTime - startTime; // Calcula o tempo decorrido
        
        Serial.print("Tempo de computação do LQR: ");
        Serial.print(executionTime);
        Serial.println(" microsegundos");
        Serial.print("Tempo em milisegundos: ");
        Serial.print(executionTime / 1000.0, 6);
        Serial.println(" ms");
        
        Serial.println("LQR gains computed successfully");

        // Export the computed gains
        float exportedGains[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(exportedGains);
        Serial.println("Exported Gains:");
        for (int i = 0; i < CONTROL_SIZE; i++) {
            for (int j = 0; j < STATE_SIZE; j++) {
                Serial.print(exportedGains[i * STATE_SIZE + j], 6);
                Serial.print(" ");
            }
            Serial.println();
        }

    } else {
        Serial.println("Failed to compute LQR gains");
    }
}

void loop(){}
