#include <AutoLQR.h>
#include "MPU9250.h"

#define STATE_SIZE 6
#define CONTROL_SIZE 3

unsigned long max_exectuion_time = 0; // Tempo máximo de execução em microssegundos

void updateSystemMatrix(float p, float q, float r);
void printGains();
void displayIMU();

const float Ixx = 0.00184;  // Momento de inércia em torno do eixo x
const float Iyy = 0.00225;  // Momento de inércia em torno do eixo y   
const float Izz = 0.00338;  // Momento de inércia em torno do eixo z
const float Ir = 0.00001;   // Momento de inércia do conjunto do motor e hélice
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

MPU9250 IMU(SPI, 5);
int status;

void setup()
{
    Serial.begin(115200);

    SPI.begin();
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }

    // setting the accelerometer full scale range to +/-8G 
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);

    // Parâmetros para discretização
    float samplingTime = 0.001; // Tempo de amostragem em segundos

    // Discretiza a matriz B
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

    IMU.readSensor();

    // Atualiza a matriz do sistema com os valores atuais
    float p = IMU.getGyroX_rads();
    float q = IMU.getGyroY_rads(); 
    float r = IMU.getGyroZ_rads(); 
    //updateSystemMatrix(p, q, r);  
    updateSystemMatrix(0, 0, 0);  

    // Calcula os ganhos ótimos
    controller.computeGains();

    unsigned long endTime = micros(); // Captura o tempo final
    unsigned long executionTime = endTime - startTime; // Calcula o tempo decorrido


    if (executionTime > max_exectuion_time & executionTime < 500000) {
        max_exectuion_time = executionTime;
    }
    
    
    Serial.print("Tempo_execucao:");
    Serial.println(executionTime);
    Serial.print(",");
    Serial.print("Tempo_Maximo:");
    Serial.println(max_exectuion_time);

    printGains();

    delay(1000); // Aguarda 1 segundo para a próxima iteração

    /*
    printGains();
        
    displayIMU();
    */


}

void updateSystemMatrix(float p, float q, float r) {
    // Update the continuous-time A matrix with current angular velocities
    // The first 3 rows remain constant
    
    // Row 4 (index 3)
    A[3 * STATE_SIZE + 4] = ((Iyy - Izz) / (2 * Ixx)) * r - Ir * omega_r / Ixx;
    A[3 * STATE_SIZE + 5] = ((Iyy - Izz) / (2 * Ixx)) * q;
    
    // Row 5 (index 4)
    A[4 * STATE_SIZE + 3] = ((Izz - Ixx) / (2 * Iyy)) * r - Ir * omega_r / Iyy;
    A[4 * STATE_SIZE + 5] = ((Izz - Ixx) / (2 * Iyy)) * p;
    
    // Row 6 (index 5)
    A[5 * STATE_SIZE + 3] = ((Ixx - Iyy) / (2 * Izz)) * q;
    A[5 * STATE_SIZE + 4] = ((Ixx - Iyy) / (2 * Izz)) * p;
    
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

void printGains(){
    Serial.println("Ganhos do LQR calculados com sucesso");

    // Exporta os ganhos calculados
    float exportedGains[CONTROL_SIZE * STATE_SIZE];
    controller.exportGains(exportedGains);
    Serial.println("Ganhos Exportados (K):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            Serial.print(exportedGains[i * STATE_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Exporta e imprime a matriz Kr
    float exportedKr[CONTROL_SIZE * CONTROL_SIZE];
    if (controller.exportKr(exportedKr)) {
        Serial.println("Matriz Kr (ganho de referência):");
        for (int i = 0; i < CONTROL_SIZE; i++) {
            for (int j = 0; j < CONTROL_SIZE; j++) {
                Serial.print(exportedKr[i * CONTROL_SIZE + j], 6);
                Serial.print(" ");
            }
            Serial.println();
        }
    } else {
        Serial.println("Não foi possível exportar a matriz Kr");
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