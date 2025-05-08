#include <AutoLQR.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>

#define STATE_SIZE 6
#define CONTROL_SIZE 3

unsigned long max_exectuion_time = 0; // Tempo máximo de execução em microssegundos

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r);
void printGains(float* K, float* Kr);
void displayIMU();

const float Ixx = 0.00184;  // Momento de inércia em torno do eixo x
const float Iyy = 0.00225;  // Momento de inércia em torno do eixo y   
const float Izz = 0.00338;  // Momento de inércia em torno do eixo z
const float Ir = 0.00001;   // Momento de inércia do conjunto do motor e hélice
float omega_r = 2500; // Velocidade angular do motor

// Variáveis para armazenar dados do sensor
float ax, ay, az; // Acelerômetro (g)
float gx, gy, gz; // Giroscópio (rad/s)
float mx, my, mz; // Magnetômetro (uT)

float Ad[STATE_SIZE * STATE_SIZE];
float Bd[STATE_SIZE * CONTROL_SIZE];

unsigned long prev_ms = micros();

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
Madgwick filter;
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

    /*
    Serial.println("Calibrando acelerometro");
    IMU.calibrateAccel();
    Serial.println("Calibrando giroscopio");
    IMU.calibrateGyro();
    Serial.println("Calibrando magnetometro");
    IMU.calibrateMag();
    */

    // Parâmetros para discretização
    float samplingTime = 0.01; // Tempo de amostragem em segundos

    // Discretiza a matriz B
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Bd[i * CONTROL_SIZE + j] = B[i * CONTROL_SIZE + j] * samplingTime;
        }
    }

    // Inicializa o controlador com a dinâmica do sistema
    updateSystemMatrix(0, 0, 0, 0, 0, 0); // Atualiza a matriz do sistema com os valores iniciais
    controller.setInputMatrix(Bd);
    controller.setCostMatrices(Q, R);

    filter.begin(1.0f/samplingTime);
}

void loop(){

    unsigned long startTime = micros(); // Captura o tempo inicial

    IMU.readSensor();

    // Converte os dados para unidades adequadas
    ax = IMU.getAccelX_mss() / 9.81; // m/s² para g
    ay = IMU.getAccelY_mss() / 9.81;
    az = -IMU.getAccelZ_mss() / 9.81;

    gx = IMU.getGyroX_rads(); // Já em rad/s
    gy = IMU.getGyroY_rads();
    gz = IMU.getGyroZ_rads();

    mx = IMU.getMagX_uT(); // Magnetômetro em µT
    my = IMU.getMagY_uT();
    mz = IMU.getMagZ_uT();

    // Atualiza o filtro de Madgwick
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // Obtém os ângulos de Euler (em graus)
    float roll = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw = filter.getYawRadians();

    float p = gx + (gz*cos(roll) + gy*sin(roll))*tan(pitch);
    float q = gy*cos(roll) + gz*sin(roll);
    float r = (gz*cos(roll) + gy*sin(roll))/cos(pitch);
    

    updateSystemMatrix(roll, pitch, yaw, p, q, r);  
    //updateSystemMatrix(0, 0, 0);  

    // Calcula os ganhos ótimos
    controller.computeGains();

    float exportedGains[CONTROL_SIZE * STATE_SIZE];
    controller.exportGains(exportedGains);

    float exportedKr[CONTROL_SIZE * CONTROL_SIZE];
    controller.exportKr(exportedKr);

    unsigned long endTime = micros(); // Captura o tempo final
    unsigned long executionTime = endTime - startTime; // Calcula o tempo decorrido

    if (executionTime > max_exectuion_time & executionTime < 50000) {
        max_exectuion_time = executionTime;
    }
    
    if(micros() >= prev_ms + 1000000){
        Serial.print("Tempo_execucao:");
        Serial.println(executionTime);
        Serial.print(",");
        Serial.print("Tempo_Maximo:");
        Serial.println(max_exectuion_time);
    
        // Exibe os resultados
        Serial.print("Roll: "); Serial.print(filter.getRoll());
        Serial.print(" | Pitch: "); Serial.print(filter.getPitch());
        Serial.print(" | Yaw: "); Serial.println(filter.getYaw());
    
        //printGains(exportedGains, exportedKr);
            
        //displayIMU();

        prev_ms = micros();
    }
}

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r) {
    // Update the continuous-time A matrix with current angular velocities
    // The first 3 rows remain constant

    A[0 * STATE_SIZE + 3] = 1;
    A[0 * STATE_SIZE + 4] = sin(roll)*tan(pitch);
    A[0 * STATE_SIZE + 5] = cos(roll)*tan(pitch);

    A[1 * STATE_SIZE + 4] = cos(roll);
    A[1 * STATE_SIZE + 5] = -sin(roll);

    A[2 * STATE_SIZE + 4] = sin(roll)/cos(pitch);
    A[2 * STATE_SIZE + 5] = cos(roll)/cos(pitch);
    
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
    float samplingTime = 0.01; // Same sampling time as in setup()
    
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

void printGains(float* K, float* Kr){
    Serial.println("Ganhos do LQR calculados com sucesso");

    // Exporta os ganhos calculados
    Serial.println("Ganhos Exportados (K):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            Serial.print(K[i * STATE_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Exporta e imprime a matriz Kr

    Serial.println("Matriz Kr (ganho de referência):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Serial.print(Kr[i * CONTROL_SIZE + j], 6);
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