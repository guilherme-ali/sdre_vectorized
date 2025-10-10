#include <AutoLQR.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>
#include "KalmanFilter.h"
#include <Wire.h>

#include "sensor_config.h" // Inclui a configuração do sensor PRIMEIRO

#ifdef USE_MPU9250
    #include <Adafruit_BMP280.h>
#endif

#ifdef USE_MPU6050
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
#endif

#include "utils.h" // Deve vir DEPOIS das definições de sensor

#define STATE_SIZE 6
#define CONTROL_SIZE 3
#define MEASUREMENT_DIM 6
#define gravity 9.81f

unsigned long max_exectuion_time = 0;
unsigned long total_execution_time = 0;
unsigned long execution_count = 0;

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r);

const float Ixx = 0.00184;
const float Iyy = 0.00225;
const float Izz = 0.00338;
const float Ir = 0.00001;
const float m = 1;
float omega_r = 1000;
const float MOTOR_B_COEFF = 0.001;
const float MOTOR_D_COEFF = 0.001;

// Variáveis para armazenar dados do sensor
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float rvx = 0, rvy = 0, rvz = 0;
float evx = 0, evy = 0, evz = 0;

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

float Q[STATE_SIZE * STATE_SIZE] = {
    100, 0, 0, 0, 0, 0,
    0, 100, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// Declaração dos sensores
#ifdef USE_MPU9250
    MPU9250 IMU(Wire, 0x68);
    Adafruit_BMP280 bmp; // BMP280 só está disponível com MPU9250
#else
    Adafruit_MPU6050 mpu;
#endif

Madgwick filter;

// Variáveis de calibração do MPU6050
#ifdef USE_MPU6050
    float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
    float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
#endif

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Inicialização dos sensores
    #ifdef USE_MPU9250
        Serial.println("Usando MPU9250 + BMP280");
        start_IMU_MPU9250(IMU);
        start_BMP(bmp);
    #else
        Serial.println("Usando MPU6050 (sem BMP280)");
        start_IMU_MPU6050(mpu);
        
        // Calibra o sensor (comente esta linha após calibrar uma vez)
        calibrate_MPU6050(mpu, accel_offset_x, accel_offset_y, accel_offset_z,
                          gyro_offset_x, gyro_offset_y, gyro_offset_z);
        
        // OU use valores fixos de calibração anteriores:
        // accel_offset_x = 0.123456;
        // accel_offset_y = -0.234567;
        // accel_offset_z = 0.345678;
        // gyro_offset_x = 0.001234;
        // gyro_offset_y = -0.002345;
        // gyro_offset_z = 0.003456;
    #endif

    // Parâmetros para discretização
    float samplingTime = 0.01;

    // Discretiza a matriz B
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Bd[i * CONTROL_SIZE + j] = B[i * CONTROL_SIZE + j] * samplingTime;
        }
    }

    // Inicializa o controlador
    updateSystemMatrix(0, 0, 0, 0, 0, 0);
    controller.setInputMatrix(Bd);
    controller.setCostMatrices(Q, R);

    filter.begin(0.01f/samplingTime);
    
    Serial.println("Sistema inicializado com sucesso!");
    Serial.println("--------------------------------------------------");
}

void loop(){
    unsigned long startTime = micros();

    // Leitura do sensor selecionado
    #ifdef USE_MPU9250
        read_MPU9250(IMU, ax, ay, az, gx, gy, gz, mx, my, mz);
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    #else
        read_MPU6050(mpu, ax, ay, az, gx, gy, gz,
                     accel_offset_x, accel_offset_y, accel_offset_z,
                     gyro_offset_x, gyro_offset_y, gyro_offset_z);
        mx = 0; my = 0; mz = 0;
        filter.updateIMU(gx, gy, gz, ax, ay, az);
    #endif

    // Obtém os ângulos de Euler
    float roll = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw = filter.getYawRadians();

    float p = gx + (gz*cos(roll) + gy*sin(roll))*tan(pitch);
    float q = gy*cos(roll) + gz*sin(roll);
    float r = (gz*cos(roll) + gy*sin(roll))/cos(pitch);

    float z_measurement[STATE_SIZE] = {roll, pitch, yaw, p, q, r};

    updateSystemMatrix(roll, pitch, yaw, p, q, r);

    // Calcula os ganhos ótimos
    controller.computeGains();

    evx = rvx - 0;
    evy = rvy - 0;
    evz = rvz - 0;

    float theta_desired = atan(evx / (evz + gravity));
    float phi_desired = -atan((evy*cos(theta_desired)) / (evz + gravity));
    float thrust = m * (evz + gravity) / (cos(theta_desired) * cos(phi_desired));

    float x[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    controller.updateState(x);

    float ref[3] = {phi_desired, theta_desired, 0};
    controller.updateReference(ref);
    
    float u[CONTROL_SIZE];
    controller.calculateControl(u);

    unsigned long endTime = micros();
    unsigned long executionTime = endTime - startTime;

    if (executionTime > max_exectuion_time & executionTime < 50000) {
        max_exectuion_time = executionTime;
    }
    
    total_execution_time += executionTime;
    execution_count++;

    if((micros() >= prev_ms + 1000000) & true){
        float avg_execution_time = (execution_count > 0) ? 
                                  (float)total_execution_time / execution_count : 0;
        
        Serial.print("Tempo_execucao:");
        Serial.println(executionTime);
        Serial.print("Tempo_Maximo:");
        Serial.println(max_exectuion_time);
        Serial.print("Tempo_Medio:");
        Serial.println(avg_execution_time);
        
        displayStates(const_cast<float*>(z_measurement));
        displayControlSignals(u, thrust);
        displayMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF);
        
        #ifdef USE_MPU9250
            //displayBMP(bmp); // BMP280 só disponível com MPU9250
        #endif
        //displayIMU(ax*9.81, ay*9.81, az*9.81, gx, gy, gz, mx, my, mz, 0);
        //displayGains();

        Serial.println("--------------------------------------------------");
        
        prev_ms = micros();
    }
}

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r) {
    // Atualiza a matriz A contínua com as velocidades angulares atuais
    // As três primeiras linhas permanecem constantes

    float sin_roll = sin(roll);
    float cos_roll = cos(roll);
    float cos_pitch = cos(pitch);
    float tan_pitch = tan(pitch);
    float inv_cos_pitch = 1.0f / cos_pitch;
    
    A[0 * STATE_SIZE + 3] = 1;
    A[0 * STATE_SIZE + 4] = sin_roll * tan_pitch;
    A[0 * STATE_SIZE + 5] = cos_roll * tan_pitch;

    A[1 * STATE_SIZE + 4] = cos_roll;
    A[1 * STATE_SIZE + 5] = -sin_roll;

    A[2 * STATE_SIZE + 4] = sin_roll * inv_cos_pitch;
    A[2 * STATE_SIZE + 5] = cos_roll * inv_cos_pitch;
    
    // Linha 4 (índice 3)
    A[3 * STATE_SIZE + 4] = ((Iyy - Izz) / (2 * Ixx)) * r - Ir * omega_r / Ixx;
    A[3 * STATE_SIZE + 5] = ((Iyy - Izz) / (2 * Ixx)) * q;
    
    // Linha 5 (índice 4)
    A[4 * STATE_SIZE + 3] = ((Izz - Ixx) / (2 * Iyy)) * r - Ir * omega_r / Iyy;
    A[4 * STATE_SIZE + 5] = ((Izz - Ixx) / (2 * Iyy)) * p;
    
    // Linha 6 (índice 5)
    A[5 * STATE_SIZE + 3] = ((Ixx - Iyy) / (2 * Izz)) * q;
    A[5 * STATE_SIZE + 4] = ((Ixx - Iyy) / (2 * Izz)) * p;
    
    // Discretiza a matriz A atualizada
    float samplingTime = 0.01; // Mesmo tempo de amostragem do setup()

    // Calcula A^2 para melhor aproximação da discretização
    float A2[STATE_SIZE * STATE_SIZE] = {0};
    MatrixOperations::matrixMultiply(A, A, A2, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    
    // Ad = I + A*dt + (A^2*dt^2)/2 (aproximação de Taylor de 2ª ordem)
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            if (i == j) {
                Ad[i * STATE_SIZE + j] = 1 + A[i * STATE_SIZE + j] * samplingTime + 
                                       A2[i * STATE_SIZE + j] * samplingTime * samplingTime * 0.5f;
            } else {
                Ad[i * STATE_SIZE + j] = A[i * STATE_SIZE + j] * samplingTime + 
                                       A2[i * STATE_SIZE + j] * samplingTime * samplingTime * 0.5f;
            }
        }
    }

    // Discretiza a matriz B com aproximação de segunda ordem: Bd = (I*dt + A*dt^2/2)*B = B*dt + A*B*dt^2/2
    float AB[STATE_SIZE * CONTROL_SIZE] = {0};
    MatrixOperations::matrixMultiply(A, B, AB, STATE_SIZE, STATE_SIZE, CONTROL_SIZE);

    float dt = samplingTime;
    float dt2_over_2 = dt * dt * 0.5f;

    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            int index = i * CONTROL_SIZE + j;
            Bd[index] = B[index] * dt + AB[index] * dt2_over_2;
        }
    }

    // Atualiza o controlador com a nova matriz de estados
    controller.setStateMatrix(Ad);
}

