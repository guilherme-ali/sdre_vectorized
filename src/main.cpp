#include <AutoLQR.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>
#include "KalmanFilter.h"
#include <Wire.h>
#include "utils.h" // Adicionar include do arquivo auxiliar

#define STATE_SIZE 6
#define CONTROL_SIZE 3
#define MEASUREMENT_DIM 6
#define gravity 9.81f

unsigned long max_exectuion_time = 0; // Tempo máximo de execução em microssegundos
unsigned long total_execution_time = 0; // Tempo total de execução
unsigned long execution_count = 0; // Contador de execuções

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r);

const float Ixx = 0.00184;  // Momento de inércia em torno do eixo x
const float Iyy = 0.00225;  // Momento de inércia em torno do eixo y   
const float Izz = 0.00338;  // Momento de inércia em torno do eixo z
const float Ir = 0.00001;   // Momento de inércia do conjunto do motor e hélice
const float m = 1;
float omega_r = 1000; // Velocidade angular do motor
const float MOTOR_B_COEFF = 0.001; // Exemplo: Coeficiente de empuxo (ex: N/(rad/s)^2)
const float MOTOR_D_COEFF = 0.001; // Exemplo: Coeficiente de arrasto/torque de guinada (ex: Nm/(rad/s)^2)

// Variáveis para armazenar dados do sensor
float ax, ay, az; // Acelerômetro (g)
float gx, gy, gz; // Giroscópio (rad/s)
float mx, my, mz; // Magnetômetro (uT)

float rvx = 0, rvy = 0, rvz = 0; // Referência de velocidade
float evx = 0, evy = 0, evz = 0; // Erro de velocidade

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

// Q_kf: Incerteza do modelo do processo. Quão confiável é nosso modelo matemático?
float Q_kf[STATE_SIZE * STATE_SIZE] = {
    0.001, 0, 0, 0, 0, 0,
    0, 0.001, 0, 0, 0, 0,
    0, 0, 0.001, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0.01
};

// R_kf: Incerteza da medição. Quão confiáveis são os dados do Madgwick/Giroscópio?
float R_kf[MEASUREMENT_DIM * MEASUREMENT_DIM] = {
    0.05, 0, 0, 0, 0, 0,
    0, 0.05, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0.1
};

// C_kf: Matriz de medição. Mapeia o estado para a medição. Como medimos todos os 6 estados, é a identidade.
float C_kf[MEASUREMENT_DIM * STATE_SIZE] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

// Controlador e variáveis de estado
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// Instância do Filtro de Kalman
KalmanFilter kf(STATE_SIZE, CONTROL_SIZE, MEASUREMENT_DIM);

float x0_kf[STATE_SIZE] = {0};
float P0_kf[STATE_SIZE * STATE_SIZE] = {0}; // Será inicializado como identidade

MPU9250 IMU(Wire, 0x68); // Alterado para usar I2C (Wire) e o endereço padrão 0x68
Madgwick filter;
int status;

void setup()
{
    Serial.begin(115200);

    Wire.begin(); // Inicializa a comunicação I2C
    status = IMU.begin();
    if (status < 0) {
        Serial.println("Falha na inicialização da IMU");
        Serial.print("Status: ");
        Serial.println(status);
        if(status != -5){
            while(1) {}
        }
    }

    // Define o range do acelerômetro para +/-8G 
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // Define o range do giroscópio para +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    // Define a largura de banda do DLPF para 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);

    /*
    Serial.println("Calibrando acelerômetro");
    IMU.calibrateAccel();
    Serial.println("Calibrando giroscópio");
    IMU.calibrateGyro();
    */
    /*
    Serial.println("Calibrando magnetômetro");
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

    filter.begin(0.01f/samplingTime);

    //Inicializa o Filtro de Kalman
    MatrixOperations::matrixIdentity(P0_kf, STATE_SIZE); // Incerteza inicial alta
    kf.init(Ad, Bd, C_kf, Q_kf, R_kf, x0_kf, P0_kf); // Usa Ad e Bd iniciais
    kf.predict({0});
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

    // Atualiza o filtro de Madgwick 700 microsegundos
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // Obtém os ângulos de Euler (em radianos)
    float roll = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw = filter.getYawRadians();

    float p = gx + (gz*cos(roll) + gy*sin(roll))*tan(pitch);
    float q = gy*cos(roll) + gz*sin(roll);
    float r = (gz*cos(roll) + gy*sin(roll))/cos(pitch);

    float z_measurement[STATE_SIZE];

    z_measurement[0] = roll;
    z_measurement[1] = pitch;
    z_measurement[2] = yaw;
    z_measurement[3] = p;
    z_measurement[4] = q;
    z_measurement[5] = r;

    kf.update(z_measurement);

    const float* filtered_state = kf.getState();

    updateSystemMatrix(filtered_state[0], filtered_state[1], filtered_state[2],
                       filtered_state[3], filtered_state[4], filtered_state[5]);

    float roll_madgwick = roll;  // Roll do Madgwick
    float roll_kalman = filtered_state[0];  // Roll do Kalman

    //updateSystemMatrix(roll, pitch, yaw, p, q, r);  

    // Calcula os ganhos ótimos
    controller.computeGains();

    evx = rvx - 0; // Referência de velocidade desejada (0 para primeiro teste)
    evy = rvy - 0;
    evz = rvz - 0;

    float theta_desired = atan(evx / (evz + gravity));
    float phi_desired = -atan((evy*cos(theta_desired) )/ (evz + gravity));
    float thrust = m * (evz + gravity) / (cos(theta_desired) * cos(phi_desired));

    // Atualiza o estado do sistema e referência
    float x[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    controller.updateState(x);

    float ref[3] = {phi_desired, theta_desired, 0};
    controller.updateReference(ref);
    
    float u[CONTROL_SIZE];
    controller.calculateControl(u);

    //A predição usa o controle calculado no *ciclo anterior*
    kf.predict(u);

    unsigned long endTime = micros(); // Captura o tempo final
    unsigned long executionTime = endTime - startTime; // Calcula o tempo decorrido

    // Atualiza o tempo máximo de execução
    if (executionTime > max_exectuion_time & executionTime < 50000) {
        max_exectuion_time = executionTime;
    }
    
    // Atualiza o tempo total de execução e o contador para cálculo da média
    total_execution_time += executionTime;
    execution_count++;

    if((micros() >= prev_ms + 1000000) & true){
        // Calcula o tempo médio de execução
        float avg_execution_time = (execution_count > 0) ? 
                                  (float)total_execution_time / execution_count : 0;
        
        Serial.print("Tempo_execucao:");
        Serial.println(executionTime);
        Serial.print("Tempo_Maximo:");
        Serial.println(max_exectuion_time);
        Serial.print("Tempo_Medio:");
        Serial.println(avg_execution_time);
        
        
        // Exibe os resultados
        displayStates(const_cast<float*>(filtered_state));
        
        //displayGains();

        displayControlSignals(u, thrust); 

        displayMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF);
            
        //displayIMU();

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
    
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            if (i == j) {
                Ad[i * STATE_SIZE + j] = 1 + A[i * STATE_SIZE + j] * samplingTime;
            } else {
                Ad[i * STATE_SIZE + j] = A[i * STATE_SIZE + j] * samplingTime;
            }
        }
    }

    // Atualiza o controlador com a nova matriz de estados
    controller.setStateMatrix(Ad);
}

