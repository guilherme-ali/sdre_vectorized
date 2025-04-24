#include <AutoLQR.h>
#include "MPU9250.h"

#define STATE_SIZE 6
#define CONTROL_SIZE 3


void print_roll_pitch_yaw();
void updateSystemMatrix(float p_angle, float q_angle, float r_angle);

const float Ixx = 0.00184;  // Momento de inércia em torno do eixo x
const float Iyy = 0.00225;  // Momento de inércia em torno do eixo y   
const float Izz = 0.00338;  // Momento de inércia em torno do eixo z
const float Ir = 0.0001;   // Momento de inércia do conjunto do motor e hélice
float omega_r = 2500; // Velocidade angular do motor

float Ad[STATE_SIZE * STATE_SIZE];
float Bd[STATE_SIZE * CONTROL_SIZE];

// Matrizes do sistema
float A[STATE_SIZE * STATE_SIZE] = {
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0
};

float B[STATE_SIZE * CONTROL_SIZE] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    1/Ixx, 0, 0,
    0, 1/Iyy, 0,
    0, 0, 1/Izz
};

// Matrizes de custo
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

// Controlador e variáveis de estado
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// Inicializa o MPU9250
MPU9250 mpu;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("Falha na conexão do MPU. Por favor, verifique sua conexão.");
        }
    }

    // Parâmetros para discretização
    float samplingTime = 0.001; // Tempo de amostragem em segundos

    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Bd[i * CONTROL_SIZE + j] = B[i * CONTROL_SIZE + j] * samplingTime;
        }
    }

    // Inicializa o controlador com a dinâmica do sistema
    updateSystemMatrix(0, 0, 0); // Atualiza a matriz do sistema com os valores iniciais
    controller.setInputMatrix(Bd);
    controller.setCostMatrices(Q, R);
}

void loop(){

    unsigned long startTime = micros(); // Captura o tempo inicial

    if (mpu.update()) {
        static uint32_t prev_ms = micros();
        if (micros() > prev_ms + 30000) { 
            print_roll_pitch_yaw();

            // Atualiza a matriz do sistema com os valores atuais
            float p = mpu.getGyroX() * DEG_TO_RAD; // Converte de graus para radianos
            float q = mpu.getGyroY() * DEG_TO_RAD; // Converte de graus para radianos
            float r = mpu.getGyroZ() * DEG_TO_RAD; // Converte de graus para radianos
            updateSystemMatrix(p, q, r);
            prev_ms = micros();
        }
    }

    // Calcula os ganhos ótimos
    if (controller.computeGains()) {
        

        unsigned long endTime = micros(); // Captura o tempo final
        unsigned long executionTime = endTime - startTime; // Calcula o tempo decorrido
        
        Serial.print("Tempo de computação do LQR: ");
        Serial.print(executionTime);
        Serial.println(" us");
        
        Serial.println("Ganhos do LQR calculados com sucesso");

        // Exporta os ganhos calculados
        float exportedGains[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(exportedGains);
        Serial.println("Ganhos Exportados:");
        for (int i = 0; i < CONTROL_SIZE; i++) {
            for (int j = 0; j < STATE_SIZE; j++) {
                Serial.print(exportedGains[i * STATE_SIZE + j], 6);
                Serial.print(" ");
            }
            Serial.println();
        }
    } 
    //delay(1000); 

}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getEulerX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getEulerY(), 2);
    Serial.print(", ");
    Serial.println(mpu.getEulerZ(), 2);
}

void updateSystemMatrix(float p_angle, float q_angle, float r_angle) {
    // Update the continuous-time A matrix with current angular velocities
    // The first 3 rows remain constant
    
    // Row 4 (index 3)
    A[3 * STATE_SIZE + 4] = ((Iyy - Izz) / (2 * Ixx)) * r_angle - Ir * omega_r / Ixx;
    A[3 * STATE_SIZE + 5] = ((Iyy - Izz) / (2 * Ixx)) * q_angle;
    
    // Row 5 (index 4)
    A[4 * STATE_SIZE + 3] = ((Izz - Ixx) / (2 * Iyy)) * r_angle - Ir * omega_r / Iyy;
    A[4 * STATE_SIZE + 5] = ((Izz - Ixx) / (2 * Iyy)) * p_angle;
    
    // Row 6 (index 5)
    A[5 * STATE_SIZE + 3] = ((Ixx - Iyy) / (2 * Izz)) * q_angle;
    A[5 * STATE_SIZE + 4] = ((Ixx - Iyy) / (2 * Izz)) * p_angle;
    
    // Discretize the updated A matrix
    float samplingTime = 0.001; // Same sampling time as in setup()
    
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            if (i == j) {
                Ad[i * STATE_SIZE + j] = 1 + A[i * STATE_SIZE + j] * samplingTime;
            } else {
                Ad[i * STATE_SIZE + j] = A[i * STATE_SIZE + j] * samplingTime;
            }
        }
    }
    
    // Update the controller with the new state matrix
    controller.setStateMatrix(Ad);
}