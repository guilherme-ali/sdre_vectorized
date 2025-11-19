#include <AutoLQR.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>
#include "KalmanFilter.h"
#include <Wire.h>

#include "sensor_config.h" 

#ifdef USE_MPU9250
    #include <Adafruit_BMP280.h>
#endif

#ifdef USE_MPU6050
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
#endif

#include "utils.h" // Deve vir DEPOIS das definições de sensor
#include "MotorControl.h" // Incluir controle de motores
#include "WiFiComm.h" // Comunicação WiFi/UDP
#include "led_control.h" // Controle de LEDs e bateria

#define STATE_SIZE 6
#define CONTROL_SIZE 3
#define MEASUREMENT_DIM 6
#define gravity 9.81f

unsigned long max_exectuion_time = 0;
unsigned long total_execution_time = 0;
unsigned long execution_count = 0;

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r);

const float Ixx = 2.1e-5;  // 0.000021 kg·m² (roll)
const float Iyy = 2.1e-5;  // 0.000021 kg·m² (pitch)
const float Izz = 3.2e-5;  // 0.000032 kg·m² (yaw)
const float Ir = 1.0e-6;   // 0.000001 kg·m² (inércia do rotor)
const float m = 0.040;     // 40g
float omega_r = 0;
const float MOTOR_B_COEFF = 3.9e-5;   // Coeficiente de empuxo (thrust): T = b*ω² [N/(rad/s)²] 3.9e-8
const float MOTOR_D_COEFF = 0.05 * MOTOR_B_COEFF;  // Coeficiente de arrasto (drag): Q = d*ω² [N·m/(rad/s)²]

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
    // VALORES DE CALIBRAÇÃO - Cole aqui os valores obtidos do script de calibração
    float accel_offset_x = 0.058127f;  // SUBSTITUIR com valor calibrado
    float accel_offset_y = -0.148659f;  // SUBSTITUIR com valor calibrado
    float accel_offset_z = 0.018737f;  // SUBSTITUIR com valor calibrado
    float gyro_offset_x = -0.011538f;   // SUBSTITUIR com valor calibrado
    float gyro_offset_y = 0.015284f;   // SUBSTITUIR com valor calibrado
    float gyro_offset_z = 0.017474f;   // SUBSTITUIR com valor calibrado
#endif

// Instância do controlador de motores
MotorControl motors;

// Instância do controle de LEDs e bateria
LEDControl leds;

// Instância da comunicação WiFi
WiFiComm wifiComm("ESP-DRONE", "12345678", 2390);

// Variáveis de controle remoto
bool remote_control_enabled = false;
CommanderPacket remote_command;

// Flags de segurança
bool enable_motors = false; // Será ativado quando controle conectar
bool motors_armed_by_remote = false; // Indica se motores foram armados pelo controle

// Callbacks para comunicação WiFi
void onRemoteCommandReceived(CommanderPacket cmd) {
    remote_command = cmd;
    remote_control_enabled = true;
    
    // Arma os motores na primeira vez que receber comando
    if (!motors_armed_by_remote && !motors.isArmed()) {
        motors.armMotors();
        motors_armed_by_remote = true;
        Serial.println("⚡ Motores ARMADOS pelo controle remoto!");
    }
}

void onClientConnected() {
    Serial.println("🎮 Controle remoto CONECTADO!");
    remote_control_enabled = false; // Aguarda primeiro comando
    enable_motors = true; // Habilita sistema de motores
    
    // Atualiza LEDs
    leds.setSystemReady(true); // Sistema pronto: LED azul piscando
}

void onClientDisconnected() {
    Serial.println("🎮 Controle remoto DESCONECTADO!");
    remote_control_enabled = false;
    enable_motors = false; // Desabilita motores por segurança
    motors_armed_by_remote = false;
    
    // Para todos os motores imediatamente
    motors.stopAllMotors();
    
    // Zera comandos por segurança
    remote_command.roll = 0;
    remote_command.pitch = 0;
    remote_command.yaw = 0;
    remote_command.thrust = 0;
    
    // Atualiza LEDs
    leds.setSystemReady(false); // Sistema não está pronto
    leds.setUDPReceiving(false); // Não está recebendo UDP
    
    Serial.println("⚠️  Motores DESARMADOS por segurança!");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Inicializa o sistema de LEDs e bateria
    leds.begin();
    // LED branco acende automaticamente (hardware) quando há energia
    
    // Verifica bateria antes de continuar
    if (leds.isCriticalBattery()) {
        Serial.println("❌ BATERIA CRÍTICA! Sistema não iniciará.");
        Serial.printf("   Tensão atual: %.2fV (mínimo: %.2fV)\n", 
                     leds.getBatteryVoltage(), BATTERY_CRITICAL_VOLTAGE);
        leds.setLowPower(true); // LED vermelho aceso
        while(1) {
            leds.update();
            delay(100);
        }
    }
    
    if (leds.isLowBattery()) {
        Serial.println("⚠️  AVISO: Bateria baixa!");
        Serial.printf("   Tensão atual: %.2fV\n", leds.getBatteryVoltage());
        leds.setLowPower(true); // LED vermelho aceso
    }

    // Inicialização dos sensores
    leds.setSensorsCalibration(true); // LED azul piscando lentamente
    
    #ifdef USE_MPU9250
        Serial.println("Usando MPU9250 + BMP280");
        start_IMU_MPU9250(IMU);
        start_BMP(bmp);
    #else
        Serial.println("Usando MPU6050 (sem BMP280)");
        start_IMU_MPU6050(mpu);
    #endif
    
    leds.setSensorsCalibration(false); // Calibração concluída

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
    
    // Inicializa o sistema de controle de motores
    motors.begin();
    motors.setThrottleLimits(0, 100); // Permite uso total dos motores (0-100%)
    motors.setOmegaSqLimits(0, 10000); // Limite superior de omega² ajustado
    
    // Descomentar para calibrar ESCs (fazer apenas uma vez)
    // motors.calibrateESCs();
    
    // NÃO armar motores aqui - eles serão armados quando o controle conectar
    Serial.println("⏸️  Motores em standby - aguardando controle remoto...");
    
    // Inicializa comunicação WiFi
    wifiComm.enableDebug(false);  // Ativa mensagens de debug
    wifiComm.enableVerbose(false); // Desativa mensagens detalhadas (pode ativar para debug)
    
    if (wifiComm.begin()) {
        Serial.println("✅ WiFi/UDP iniciado com sucesso!");
        
        // Configura callbacks
        wifiComm.onCommandReceived(onRemoteCommandReceived);
        wifiComm.onClientConnected(onClientConnected);
        wifiComm.onClientDisconnected(onClientDisconnected);
    } else {
        Serial.println("❌ Falha ao iniciar WiFi/UDP!");
    }
    
    Serial.println("Sistema inicializado com sucesso!");
    Serial.println("--------------------------------------------------");
}

void loop(){
    unsigned long startTime = micros();
    
    // ===== PROFILING: Tempos parciais =====
    unsigned long t_leds, t_battery, t_wifi, t_sensor, t_filter, t_angles, t_matrix, t_lqr, t_control_logic, t_motor_calc, t_motor_set;
    unsigned long t_checkpoint = micros();
    
    // Atualiza sistema de LEDs
    leds.update();
    t_leds = micros() - t_checkpoint;
    t_checkpoint = micros();
    
    // Verifica bateria crítica - desliga motores se necessário
    if (leds.isCriticalBattery() && motors.isArmed()) {
        Serial.println("🔋 BATERIA CRÍTICA! Desarmando motores...");
        motors.stopAllMotors();
        enable_motors = false;
        leds.setLowPower(true);
    } else if (leds.isLowBattery()) {
        leds.setLowPower(true); // LED vermelho aceso
    } else {
        leds.setLowPower(false); // LED vermelho apagado
    }
    t_battery = micros() - t_checkpoint;
    t_checkpoint = micros();
    
    // Atualiza comunicação WiFi
    wifiComm.update();
    t_wifi = micros() - t_checkpoint;
    t_checkpoint = micros();
    
    // Atualiza LED de recepção UDP
    if (wifiComm.isClientConnected()) {
        leds.setUDPReceiving(true); // LED verde piscando
    } else {
        leds.setUDPReceiving(false);
    };

    // Leitura do sensor selecionado
    #ifdef USE_MPU9250
        read_MPU9250(IMU, ax, ay, az, gx, gy, gz, mx, my, mz);
        t_sensor = micros() - t_checkpoint;
        t_checkpoint = micros();
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        t_filter = micros() - t_checkpoint;
        t_checkpoint = micros();
    #else
        read_MPU6050(mpu, ax, ay, az, gx, gy, gz,
                     accel_offset_x, accel_offset_y, accel_offset_z,
                     gyro_offset_x, gyro_offset_y, gyro_offset_z);
        t_sensor = micros() - t_checkpoint;
        t_checkpoint = micros();
        mx = 0; my = 0; mz = 0;
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        t_filter = micros() - t_checkpoint;
        t_checkpoint = micros();
    #endif

    // Obtém os ângulos de Euler
    float roll = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw = filter.getYawRadians();
    yaw = 0 ; // Zera o yaw para evitar deriva

    float p = gx + (gz*cos(roll) + gy*sin(roll))*tan(pitch);
    float q = gy*cos(roll) + gz*sin(roll);
    float r = (gz*cos(roll) + gy*sin(roll))/cos(pitch);

    float z_measurement[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    t_angles = micros() - t_checkpoint;
    t_checkpoint = micros();

    updateSystemMatrix(roll, pitch, yaw, p, q, r);
    t_matrix = micros() - t_checkpoint;
    t_checkpoint = micros();

    // Calcula os ganhos ótimos
    controller.computeGains();
    t_lqr = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ===== LÓGICA DE CONTROLE =====
    float phi_desired, theta_desired, yaw_desired, thrust;
    
    if (remote_control_enabled && wifiComm.isClientConnected()) {
        // Modo de controle remoto via WiFi
        // Converte comandos do joystick para setpoints em radianos
        // Joystick vai de -1.0 a 1.0, vamos limitar a ±30 graus (±0.524 rad)
        const float MAX_ANGLE_RAD = 0.524f; // 30 graus em radianos
        const float MAX_YAW_RATE_RAD = 1.57f; // 90 graus/s em radianos
        
        phi_desired = remote_command.roll * MAX_ANGLE_RAD;        // Roll: ±30°
        theta_desired = remote_command.pitch * MAX_ANGLE_RAD;     // Pitch: ±30°
        yaw_desired = remote_command.yaw * MAX_YAW_RATE_RAD;      // Yaw rate: ±90°/s
        
        // Thrust: converte de 0-65535 para força em Newtons
        // Valor mínimo para manter no ar: m*g = 0.04*9.81 = 0.3924 N
        // Vamos mapear thrust de 0% a 200% do peso
        thrust = (remote_command.thrust / 65535.0f) * m * gravity * 2.0f;
    } else {
        // Modo autônomo (código original)
        evx = rvx - 0;
        evy = rvy - 0;
        evz = rvz - 0;

        theta_desired = atan(evx / (evz + gravity));
        phi_desired = -atan((evy*cos(theta_desired)) / (evz + gravity));
        yaw_desired = 0;
        thrust = m * (evz + gravity) / (cos(theta_desired) * cos(phi_desired));
    }
    t_control_logic = micros() - t_checkpoint;
    t_checkpoint = micros();
    // ==============================

    float x[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    controller.updateState(x);

    float ref[3] = {phi_desired, theta_desired, yaw_desired};
    controller.updateReference(ref);
    
    float u[CONTROL_SIZE];
    controller.calculateControl(u);
    // Tempo do cálculo de controle já está incluído em t_lqr acima

    // ===== CÁLCULO DOS OMEGA QUADRADOS =====
    float w1_sq, w2_sq, w3_sq, w4_sq;
    calculateMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF,
                          w1_sq, w2_sq, w3_sq, w4_sq);
    t_motor_calc = micros() - t_checkpoint;
    t_checkpoint = micros();
    // ======================================

    // ===== CONTROLE DE MOTORES =====
    if (enable_motors && motors.isArmed()) {
        motors.setMotorsFromOmegaSq(w1_sq, w2_sq, w3_sq, w4_sq);
    } else {
        motors.stopAllMotors();
    }
    t_motor_set = micros() - t_checkpoint;
    // ==============================

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
        
        // ===== IMPRESSÃO CONSOLIDADA =====
        Serial.println("\n========== STATUS DO SISTEMA ==========");
        
        // ===== PROFILING DE DESEMPENHO =====
        Serial.println("\n⏱️  PROFILING DE EXECUÇÃO:");
        Serial.printf("   LEDs:            %4lu μs (%5.1f%%)\n", t_leds, (t_leds * 100.0f) / executionTime);
        Serial.printf("   Bateria:         %4lu μs (%5.1f%%)\n", t_battery, (t_battery * 100.0f) / executionTime);
        Serial.printf("   WiFi/UDP:        %4lu μs (%5.1f%%)\n", t_wifi, (t_wifi * 100.0f) / executionTime);
        Serial.printf("   Leitura Sensor:  %4lu μs (%5.1f%%)\n", t_sensor, (t_sensor * 100.0f) / executionTime);
        Serial.printf("   Filtro Madgwick: %4lu μs (%5.1f%%)\n", t_filter, (t_filter * 100.0f) / executionTime);
        Serial.printf("   Cálc. Ângulos:   %4lu μs (%5.1f%%)\n", t_angles, (t_angles * 100.0f) / executionTime);
        Serial.printf("   Matriz Sistema:  %4lu μs (%5.1f%%)\n", t_matrix, (t_matrix * 100.0f) / executionTime);
        Serial.printf("   LQR (Ganhos):    %4lu μs (%5.1f%%)\n", t_lqr, (t_lqr * 100.0f) / executionTime);
        Serial.printf("   Lógica Controle: %4lu μs (%5.1f%%)\n", t_control_logic, (t_control_logic * 100.0f) / executionTime);
        Serial.printf("   Cálc. Omega²:    %4lu μs (%5.1f%%)\n", t_motor_calc, (t_motor_calc * 100.0f) / executionTime);
        Serial.printf("   Set Motores:     %4lu μs (%5.1f%%)\n", t_motor_set, (t_motor_set * 100.0f) / executionTime);
        Serial.println("   -----------------------------------");
        
        unsigned long sum_profiled = t_leds + t_battery + t_wifi + t_sensor + t_filter + 
                                     t_angles + t_matrix + t_lqr + t_control_logic + 
                                     t_motor_calc + t_motor_set;
        unsigned long overhead = executionTime - sum_profiled;
        Serial.printf("   Soma Medida:     %4lu μs (%5.1f%%)\n", sum_profiled, (sum_profiled * 100.0f) / executionTime);
        Serial.printf("   Overhead/Outros: %4lu μs (%5.1f%%)\n", overhead, (overhead * 100.0f) / executionTime);
        
        // Tempo de execução
        Serial.println("\n📊 ESTATÍSTICAS DE TEMPO:");
        Serial.print("   Tempo_execucao: ");
        Serial.print(executionTime);
        Serial.println(" μs");
        Serial.print("   Tempo_Maximo: ");
        Serial.print(max_exectuion_time);
        Serial.println(" μs");
        Serial.print("   Tempo_Medio: ");
        Serial.print(avg_execution_time);
        Serial.println(" μs");
        
        // Status de controle
        if (remote_control_enabled && wifiComm.isClientConnected()) {
            Serial.println("\n🎮 MODO: CONTROLE REMOTO ATIVO");
            Serial.println("   Comandos Joystick:");
            Serial.printf("     Roll:   %+.3f\n", remote_command.roll);
            Serial.printf("     Pitch:  %+.3f\n", remote_command.pitch);
            Serial.printf("     Yaw:    %+.3f\n", remote_command.yaw);
            Serial.printf("     Thrust: %d (%.1f%%)\n", remote_command.thrust, 
                         (remote_command.thrust / 65535.0f) * 100.0f);
            Serial.println("   Setpoints (rad):");
            Serial.printf("     φ_desired: %+.4f\n", phi_desired);
            Serial.printf("     θ_desired: %+.4f\n", theta_desired);
            Serial.printf("     ψ_desired: %+.4f\n", yaw_desired);
            Serial.printf("     Thrust: %.4f N\n", thrust);
            Serial.println("   Erros:");
            Serial.printf("     e_φ: %+.4f rad\n", phi_desired - roll);
            Serial.printf("     e_θ: %+.4f rad\n", theta_desired - pitch);
            Serial.printf("     e_ψ: %+.4f rad\n", yaw_desired - yaw);
        } else {
            Serial.println("\n🤖 MODO: AUTÔNOMO");
        }
        
        // Estados do sistema
        Serial.println();
        displayStates(const_cast<float*>(z_measurement));
        
        // Sinais de controle
        displayControlSignals(u, thrust);
        
        // Omega dos motores
        displayMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF);
        
        // Valores dos motores
        motors.printMotorValues();
        
        // Status dos motores
        Serial.print("\n🔧 Motores: ");
        if (motors.isArmed()) {
            Serial.println("ARMADOS ✅");
        } else {
            Serial.println("DESARMADOS ⏸️");
        }
        
        // Status WiFi
        if (wifiComm.isClientConnected()) {
            Serial.print("📡 WiFi: CONECTADO | Pacotes: ");
            Serial.print(wifiComm.getPacketCount());
            Serial.print(" | Cliente: ");
            Serial.println(wifiComm.getClientIP());
        } else {
            Serial.println("📡 WiFi: Aguardando conexão...");
        }
        
        // Status da bateria e LEDs
        //Serial.println();
        //leds.printStatus();
        
        #ifdef USE_MPU9250
            //displayBMP(bmp); // BMP280 só disponível com MPU9250
        #endif
        //displayIMU(ax*9.81, ay*9.81, az*9.81, gx, gy, gz, mx, my, mz, 0);
        //displayGains();

        // Exibe a Matriz P (Solução da Equação de Riccati)
        Serial.println("\n📊 Matriz P (Solução de Riccati):");
        const float* P_ptr = controller.getRicattiSolution();
        if (P_ptr) {
            for (int i = 0; i < STATE_SIZE; i++) {
                Serial.print("   | ");
                for (int j = 0; j < STATE_SIZE; j++) {
                    Serial.print(P_ptr[i * STATE_SIZE + j], 4); // 4 casas decimais
                    if (j < STATE_SIZE - 1) Serial.print("\t");
                }
                Serial.println(" |");
            }
        }

        // Exibe a Matriz K (Ganhos de Controle)
        Serial.println("\n🎮 Matriz K (Ganhos de Controle):");
        float K_current[CONTROL_SIZE * STATE_SIZE];
        if (controller.exportGains(K_current)) {
            for (int i = 0; i < CONTROL_SIZE; i++) {
                Serial.print("   | ");
                for (int j = 0; j < STATE_SIZE; j++) {
                    Serial.print(K_current[i * STATE_SIZE + j], 4);
                    if (j < STATE_SIZE - 1) Serial.print("\t");
                }
                Serial.println(" |");
            }
        }

        Serial.println("========================================\n");
        
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

