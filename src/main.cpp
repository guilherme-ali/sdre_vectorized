/**
 * TESTE: Backup do Main - Implementação Completa do Sistema
 * 
 * Versão backup do código principal contendo a implementação completa do
 * controle de atitude do drone com sensores, filtros, comunicação WiFi e
 * controle de motores usando SDRE-LQR.
 */

#include <AutoLQR.h>
#include "PIDController.h"
#include "MPU9250.h"
#include <MadgwickAHRS.h>
#include "KalmanFilter.h"
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===== FLAG DE DEBUG =====
// Coloque true para ver prints detalhados, false para Serial Plotter
const bool DEBUG_MODE = false;
// ==========================

// ===== FLAG DO MAGNETÔMETRO =====
// Coloque true para usar QMC5883L, false para usar apenas accel+gyro (6-DOF)
const bool USE_MAGNETOMETER = false;
// =================================  

// ===== TIPO DE CONTROLADOR =====
// 0 = SDRE (State-Dependent Riccati Equation)
// 1 = PID (Proportional-Integral-Derivative)
const int CONTROLLER_TYPE = 0;
// ================================

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
#define gravity 9.80665f

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r, float omega_r);

const float Ixx = 16.57e-6;  // 0.000021 kg·m² (roll)
const float Iyy = 16.57e-6;  // 0.000021 kg·m² (pitch)
const float Izz = 29.80e-6;  // 0.000032 kg·m² (yaw)
const float Ir = 1.02e-7;   // 0.000001 kg·m² (inércia do rotor)
const float m = 0.040;     // 40g
const float L_ARM = 0.060f; // 60mm - distância do centro ao motor (braço)
const float SAMPLING_TIME_S = 0.02f; // 20 ms
const unsigned long LOOP_PERIOD_US = static_cast<unsigned long>(SAMPLING_TIME_S * 1e6f);
float omega_r = 0;

// Coeficientes do motor e hélice (VALORES MEDIDOS - teste_motor_v2)

const float MOTOR_B_COEFF = 1.11e-8f;   // Coeficiente de empuxo MEDIDO (regressão: F = 1.11E-08*ω² - 5.09E-04) [N/(rad/s)²]
const float MOTOR_D_COEFF = 0.05f * MOTOR_B_COEFF;  // Coeficiente de arrasto (drag): Q = d*ω² [N·m/(rad/s)²] (estimado)


const float MAX_RPM = 31086.0f; // RPM medido a 100% duty cycle
const float MAX_OMEGA = (MAX_RPM * 2.0f * PI) / 60.0f; // ~3255.3 rad/s
const float MAX_THRUST = MOTOR_B_COEFF * MAX_OMEGA * MAX_OMEGA; // Empuxo máximo medido a 100% duty cycle (N)

// Variáveis para armazenar dados do sensor
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float rvx = 0, rvy = 0, rvz = 0;
float evx = 0, evy = 0, evz = 0;

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
// Baseando-se em Regra de Bryson
// usando angulos maximos de 30 grus e valocidades angulares maximas de 1rad/s
// Qii = 1/(max_estado_i)^2 
float Q[STATE_SIZE * STATE_SIZE] = {
    3.65, 0, 0, 0, 0, 0,
    0, 3.65, 0, 0, 0, 0,
    0, 0, 0.91, 0, 0, 0,
    0, 0, 0, 1.0, 0, 0,
    0, 0, 0, 0, 1.0, 0,
    0, 0, 0, 0, 0, 4.0
};

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

// Controladores
AutoLQR sdreController(STATE_SIZE, CONTROL_SIZE);
PIDController pidController(STATE_SIZE, CONTROL_SIZE);

// Declaração dos sensores
#ifdef USE_MPU9250
    MPU9250 IMU(Wire, 0x68);
    Adafruit_BMP280 bmp; // BMP280 só está disponível com MPU9250
#else
    Adafruit_MPU6050 mpu;
#endif

// Referência ao segundo barramento I2C para sensores adicionais (QMC5883L magnetometer)
// SDA = GPIO40, SCL = GPIO41
// Wire1 já é definido pela biblioteca Wire do ESP32
extern TwoWire Wire1;

Madgwick filter;

// Variáveis de calibração do MPU6050
#ifdef USE_MPU6050
    // VALORES DE CALIBRAÇÃO MPU6050 - Cole aqui os valores obtidos do script de calibração
    float accel_offset_x = 0.058127f;  // SUBSTITUIR com valor calibrado
    float accel_offset_y = -0.148659f;  // SUBSTITUIR com valor calibrado
    float accel_offset_z = 0.018737f;  // SUBSTITUIR com valor calibrado
    float gyro_offset_x = -0.011538f;   // SUBSTITUIR com valor calibrado
    float gyro_offset_y = 0.015284f;   // SUBSTITUIR com valor calibrado
    float gyro_offset_z = 0.017474f;   // SUBSTITUIR com valor calibrado
#endif

// ===== CALIBRAÇÃO DO MAGNETÔMETRO QMC5883L =====
// Execute test/calibrate_magnetometer.cpp para obter estes valores
// Hard-Iron Offsets (valores brutos para subtrair)
const float MAG_OFFSET_X = 0.0f;   // SUBSTITUIR com valor calibrado
const float MAG_OFFSET_Y = 0.0f;   // SUBSTITUIR com valor calibrado
const float MAG_OFFSET_Z = 0.0f;   // SUBSTITUIR com valor calibrado
// Soft-Iron Scales (fatores de escala)
const float MAG_SCALE_X = 1.0f;    // SUBSTITUIR com valor calibrado
const float MAG_SCALE_Y = 1.0f;    // SUBSTITUIR com valor calibrado
const float MAG_SCALE_Z = 1.0f;    // SUBSTITUIR com valor calibrado
// ===============================================

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
bool skip_timing_sample = false; // Ignora amostra de tempo durante armamento

// Callbacks para comunicação WiFi
void onRemoteCommandReceived(CommanderPacket cmd) {
    remote_command = cmd;
    remote_control_enabled = true;
    
    // Arma os motores na primeira vez que receber comando
    if (!motors_armed_by_remote && !motors.isArmed()) {
        skip_timing_sample = true;
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
    Serial.println("\n🔴🔴🔴 EVENTO: onClientDisconnected() CHAMADO! 🔴🔴🔴");
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
    // Desabilita o detector de brownout para evitar reset com queda de tensão da bateria
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

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
    
    // Inicializa o magnetômetro QMC5883L no segundo barramento I2C (se habilitado)
    if (USE_MAGNETOMETER) {
        Serial.println("Inicializando QMC5883L (Magnetômetro)...");
        start_QMC5883L(Wire1);
        
        // Configura calibração do magnetômetro (valores obtidos de test/calibrate_magnetometer.cpp)
        setQMC5883LCalibration(MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z,
                               MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);
    } else {
        Serial.println("⚠️  Magnetômetro DESABILITADO - usando apenas Accel+Gyro (6-DOF)");
    }
    
    leds.setSensorsCalibration(false); // Calibração concluída

    // Discretiza a matriz B
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Bd[i * CONTROL_SIZE + j] = B[i * CONTROL_SIZE + j] * SAMPLING_TIME_S;
        }
    }

    // Inicializa o controlador baseado no tipo selecionado
    updateSystemMatrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    if (CONTROLLER_TYPE == 0) {
        // SDRE Controller
        Serial.println("🎯 Controlador: SDRE (State-Dependent Riccati Equation)");
        sdreController.setInputMatrix(Bd);
        sdreController.setCostMatrices(Q, R);
    } else {
        // PID Controller
        Serial.println("🎯 Controlador: PID (Proportional-Integral-Derivative)");
        // Configurar ganhos PID (ajustar conforme necessário)
        pidController.setRollGains(0.05f, 0.05f, 0.0f);
        pidController.setPitchGains(0.05f, 0.05f, 0.0f);
        pidController.setYawGains(0.01f, 0.0f, 0.0f);


        pidController.setIntegralLimits(1.0f, 1.0f, 0.5f);
        pidController.setOutputLimits(-10.0f, 10.0f);
        pidController.setSamplingTime(SAMPLING_TIME_S);
    }

    filter.begin(1.0f / SAMPLING_TIME_S);
    
    // Inicializa o sistema de controle de motores
    motors.begin();
    motors.setThrottleLimits(0, 100); // Permite uso total dos motores (0-100%)
    // Motor max: 50000 RPM = 5236 rad/s → ω² = 27.4M rad²/s²
    motors.setOmegaSqLimits(0, MAX_OMEGA * MAX_OMEGA); // Limite superior de omega² para 31k RPM (medido)
    
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
    unsigned long t_leds, t_battery, t_wifi, t_mpu, t_mag, t_filter, t_angles, t_matrix, t_lqr, t_control_logic, t_motor_calc, t_motor_set;
    unsigned long t_checkpoint = micros();
    
    // Atualiza sistema de LEDs
    leds.update();
    t_leds = micros() - t_checkpoint;
    t_checkpoint = micros();
    
    // Verifica bateria crítica - apenas atualiza LED (não desarma motores)
    if (leds.isCriticalBattery()) {
        leds.setLowPower(true);
    } else if (leds.isLowBattery()) {
        leds.setLowPower(true);
    } else {
        leds.setLowPower(false);
    }
    t_battery = micros() - t_checkpoint;
    t_checkpoint = micros();
    
    // Atualiza comunicação WiFi
    wifiComm.update();
    t_wifi = micros() - t_checkpoint;
    t_checkpoint = micros();
    
    // Atualiza LED de recepção UDP
    if (wifiComm.isClientConnected()) {
        leds.setUDPReceiving(true);
    } else {
        leds.setUDPReceiving(false);
    }

    // Leitura do sensor selecionado
    #ifdef USE_MPU9250
        read_MPU9250(IMU, ax, ay, az, gx, gy, gz, mx, my, mz);
        t_mpu = micros() - t_checkpoint;
        t_mag = 0; // MPU9250 tem magnetômetro integrado
        t_checkpoint = micros();
        // MPU9250 retorna rad/s, Madgwick espera graus/s
        if (USE_MAGNETOMETER) {
            filter.update(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, 
                          ax, ay, az, mx, my, mz);
        } else {
            filter.updateIMU(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, 
                             ax, ay, az);
        }
        t_filter = micros() - t_checkpoint;
        t_checkpoint = micros();
    #else
        read_MPU6050(mpu, ax, ay, az, gx, gy, gz,
                     accel_offset_x, accel_offset_y, accel_offset_z,
                     gyro_offset_x, gyro_offset_y, gyro_offset_z);
        t_mpu = micros() - t_checkpoint;
        t_checkpoint = micros();
        
        // Lê o magnetômetro QMC5883L (se habilitado)
        if (USE_MAGNETOMETER) {
            read_QMC5883L(mx, my, mz);
            t_mag = micros() - t_checkpoint;
            t_checkpoint = micros();
            // Adafruit MPU6050 retorna rad/s, Madgwick espera graus/s
            filter.update(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, 
                          ax, ay, az, mx, my, mz);
        } else {
            t_mag = 0;
            // Usa apenas accel+gyro (6-DOF) - sem magnetômetro
            filter.updateIMU(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, 
                             ax, ay, az);
        }
        t_filter = micros() - t_checkpoint;
        t_checkpoint = micros();
    #endif

    // Obtém os ângulos de Euler
    float roll = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw = filter.getYawRadians();
    yaw = 0.0f;

    float p = gx + (gz*cos(roll) + gy*sin(roll))*tan(pitch);
    float q = gy*cos(roll) + gz*sin(roll);
    float r = (gz*cos(roll) + gy*sin(roll))/cos(pitch);

    float z_measurement[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    t_angles = micros() - t_checkpoint;
    t_checkpoint = micros();

    updateSystemMatrix(roll, pitch, yaw, p, q, r, omega_r);
    t_matrix = micros() - t_checkpoint;
    t_checkpoint = micros();

    // Calcula os ganhos ótimos (somente para SDRE)
    if (CONTROLLER_TYPE == 0) {
        sdreController.computeGains();
    }
    t_lqr = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ===== LÓGICA DE CONTROLE =====
    float phi_desired, theta_desired, yaw_desired, thrust;
    
    if (remote_control_enabled && wifiComm.isClientConnected()) {
        
        phi_desired = remote_command.roll * DEG_TO_RAD;
        theta_desired = remote_command.pitch * DEG_TO_RAD;
        yaw_desired = remote_command.yaw * DEG_TO_RAD;

        thrust = (remote_command.thrust / 65535.0f) * MAX_THRUST * 4; // 4 motores
    } else {
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

    float x[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    float ref[3] = {phi_desired, theta_desired, yaw_desired};
    float u[CONTROL_SIZE];

    // Calcula controle baseado no tipo de controlador selecionado
    if (CONTROLLER_TYPE == 0) {
        // SDRE Controller
        sdreController.updateState(x);
        sdreController.updateReference(ref);
        sdreController.calculateControl(u);
    } else {
        // PID Controller
        pidController.updateState(x);
        pidController.updateReference(ref);
        pidController.calculateControl(u);
    }

    // ===== CÁLCULO DOS OMEGA QUADRADOS =====
    float w1_sq, w2_sq, w3_sq, w4_sq;
    calculateMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF, L_ARM,
                          w1_sq, w2_sq, w3_sq, w4_sq);
    // Atualiza omega_r para a próxima iteração (Voos 2006):
    // omega_r = -ω1 + ω2 - ω3 + ω4  (motores CW: 1,3 / CCW: 2,4)
    omega_r = -sqrtf(w1_sq) + sqrtf(w2_sq) - sqrtf(w3_sq) + sqrtf(w4_sq);
    t_motor_calc = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ===== CONTROLE DE MOTORES =====
    if (enable_motors && motors.isArmed()) {
        motors.setMotorsFromOmegaSq(w1_sq, w2_sq, w3_sq, w4_sq);
    } else {
        motors.stopAllMotors();
    }
    t_motor_set = micros() - t_checkpoint;

    // Tempo de processamento
    unsigned long processingTime = micros() - startTime;
    
    // Força o loop a rodar exatamente no período de amostragem configurado
    if (processingTime < LOOP_PERIOD_US) {
        delayMicroseconds(LOOP_PERIOD_US - processingTime);
    }
    
    // Tempo total de execução do loop (deve ser ~12000 μs)
    unsigned long loopTime = micros() - startTime;
    
    // Estatísticas
    static unsigned long maxTime = 0;
    static unsigned long totalTime = 0;
    static unsigned long loopCount = 0;
    static unsigned long prev_ms = 0;
    static unsigned long last_print_time = 0;  // Tempo que os prints custaram na última vez

    bool skipThisSample = skip_timing_sample;
    if (skipThisSample) {
        skip_timing_sample = false;
    }

    bool newMaxTime = false;
    if (!skipThisSample) {
        loopCount++;
        totalTime += loopTime;
        if (loopTime > maxTime) {
            maxTime = loopTime;
            newMaxTime = true;
        }
    }

    float avgTime = (loopCount > 0) ? ((float)totalTime / loopCount) : 0.0f;
    
    if (DEBUG_MODE) {
        // ===== MODO DEBUG: Prints detalhados a cada 1 segundo =====
        if (micros() >= prev_ms + 1000000) {
            unsigned long print_start = micros();
            
            // Calcula tempo total do loop incluindo prints anteriores
            unsigned long total_with_prints = processingTime + last_print_time;
            
            Serial.println("\n========== STATUS DO SISTEMA ==========");
            
            // ===== PROFILING DE DESEMPENHO =====
            Serial.println("\n⏱️  PROFILING DE EXECUÇÃO:");
            Serial.printf("   LEDs:            %4lu μs (%5.1f%%)\n", t_leds, (t_leds * 100.0f) / total_with_prints);
            Serial.printf("   Bateria:         %4lu μs (%5.1f%%)\n", t_battery, (t_battery * 100.0f) / total_with_prints);
            Serial.printf("   WiFi/UDP:        %4lu μs (%5.1f%%)\n", t_wifi, (t_wifi * 100.0f) / total_with_prints);
            Serial.printf("   Leitura MPU:     %4lu μs (%5.1f%%)\n", t_mpu, (t_mpu * 100.0f) / total_with_prints);
            Serial.printf("   Leitura Mag:     %4lu μs (%5.1f%%)\n", t_mag, (t_mag * 100.0f) / total_with_prints);
            Serial.printf("   Filtro Madgwick: %4lu μs (%5.1f%%)\n", t_filter, (t_filter * 100.0f) / total_with_prints);
            Serial.printf("   Cálc. Ângulos:   %4lu μs (%5.1f%%)\n", t_angles, (t_angles * 100.0f) / total_with_prints);
            Serial.printf("   Matriz Sistema:  %4lu μs (%5.1f%%)\n", t_matrix, (t_matrix * 100.0f) / total_with_prints);
            Serial.printf("   LQR (Ganhos):    %4lu μs (%5.1f%%)\n", t_lqr, (t_lqr * 100.0f) / total_with_prints);
            Serial.printf("   Lógica Controle: %4lu μs (%5.1f%%)\n", t_control_logic, (t_control_logic * 100.0f) / total_with_prints);
            Serial.printf("   Cálc. Omega²:    %4lu μs (%5.1f%%)\n", t_motor_calc, (t_motor_calc * 100.0f) / total_with_prints);
            Serial.printf("   Set Motores:     %4lu μs (%5.1f%%)\n", t_motor_set, (t_motor_set * 100.0f) / total_with_prints);
            Serial.printf("   Prints (ant.):   %4lu μs (%5.1f%%)\n", last_print_time, (last_print_time * 100.0f) / total_with_prints);
            Serial.println("   -----------------------------------");
            unsigned long sum_profiled = t_leds + t_battery + t_wifi + t_mpu + t_mag + t_filter + 
                                         t_angles + t_matrix + t_lqr + t_control_logic + 
                                         t_motor_calc + t_motor_set + last_print_time;
            unsigned long overhead = (total_with_prints > sum_profiled) ? (total_with_prints - sum_profiled) : 0;
            Serial.printf("   Soma Medida:     %4lu μs (%5.1f%%)\n", sum_profiled, (sum_profiled * 100.0f) / total_with_prints);
            Serial.printf("   Overhead/Outros: %4lu μs (%5.1f%%)\n", overhead, (overhead * 100.0f) / total_with_prints);
            
            // Tempo de execução
            Serial.println("\n📊 ESTATÍSTICAS DE TEMPO:");
            Serial.printf("   Tempo_Loop: %lu μs\n", loopTime);
            Serial.printf("   Tempo_Processamento: %lu μs\n", processingTime);
            Serial.printf("   Tempo_Maximo: %lu μs\n", maxTime);
            Serial.printf("   Tempo_Medio: %.2f μs\n", avgTime);
            
            // Status de controle
            if (remote_control_enabled && wifiComm.isClientConnected()) {
                Serial.println("\n🎮 MODO: CONTROLE REMOTO ATIVO");
                Serial.println("   Comandos Joystick:");
                Serial.printf("     Roll:   %+.3f\n", remote_command.roll);
                Serial.printf("     Pitch:  %+.3f\n", remote_command.pitch);
                Serial.printf("     Yaw:    %+.3f\n", remote_command.yaw);
                Serial.printf("     Thrust: %d (%.1f%%)\n", remote_command.thrust, 
                             (remote_command.thrust / 60000.0f) * 100.0f);
                Serial.println("   Setpoints (rad):");
                Serial.printf("     φ_desired: %+.4f\n", phi_desired);
                Serial.printf("     θ_desired: %+.4f\n", theta_desired);
                Serial.printf("     ψ_desired: %+.4f\n", yaw_desired);
                Serial.printf("     Thrust: %.4f N\n", thrust);
            } else {
                Serial.println("\n🤖 MODO: AUTÔNOMO");
            }
            
            // Estados do sistema
            Serial.println();
            displayStates(const_cast<float*>(z_measurement));
            
            // Dados do magnetômetro (se habilitado)
            if (USE_MAGNETOMETER) {
                Serial.println("\n🧭 MAGNETÔMETRO (QMC5883L):");
                Serial.printf("   Campo Magnético: X=%+.2f  Y=%+.2f  Z=%+.2f μT\n", mx, my, mz);
                float heading = atan2(my, mx) * RAD_TO_DEG;
                if (heading < 0) heading += 360.0f;
                Serial.printf("   Azimute (Heading): %.1f°\n", heading);
                // Intensidade total do campo magnético
                float mag_intensity = sqrt(mx*mx + my*my + mz*mz);
                Serial.printf("   Intensidade Total: %.2f μT\n", mag_intensity);
            } else {
                Serial.println("\n🧭 MAGNETÔMETRO: DESABILITADO (6-DOF mode)");
            }
            
            // Sinais de controle
            displayControlSignals(u, thrust);
            
            // Omega dos motores
            displayMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF, L_ARM);
            
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

            // Exibe a Matriz P (Solução da Equação de Riccati) - apenas para SDRE
            if (CONTROLLER_TYPE == 0) {
                Serial.println("\n📊 Matriz P (Solução de Riccati):");
                const float* P_ptr = sdreController.getRicattiSolution();
                if (P_ptr) {
                    for (int i = 0; i < STATE_SIZE; i++) {
                        Serial.print("   | ");
                        for (int j = 0; j < STATE_SIZE; j++) {
                            Serial.print(P_ptr[i * STATE_SIZE + j], 4);
                            if (j < STATE_SIZE - 1) Serial.print("\t");
                        }
                        Serial.println(" |");
                    }
                }
                
                // Exibe a Matriz K (Ganhos de Controle) - apenas para SDRE
                Serial.println("\n🎮 Matriz K (Ganhos de Controle):");
                float K_current[CONTROL_SIZE * STATE_SIZE];
                if (sdreController.exportGains(K_current)) {
                    for (int i = 0; i < CONTROL_SIZE; i++) {
                        Serial.print("   | ");
                        for (int j = 0; j < STATE_SIZE; j++) {
                            Serial.print(K_current[i * STATE_SIZE + j], 4);
                            if (j < STATE_SIZE - 1) Serial.print("\t");
                        }
                        Serial.println(" |");
                    }
                }
            } else {
                // Para PID, exibe os ganhos de forma mais apropriada
                Serial.println("\n📊 Ganhos PID:");
                Serial.println("   Eixo     |   Kp    |   Ki    |   Kd    |");
                Serial.println("   ---------|---------|---------|---------|");
                float kp, ki, kd;
                pidController.getGains(0, kp, ki, kd);  // Roll
                Serial.printf("   Roll     |  %5.2f  |  %5.2f  |  %5.2f  |\n", kp, ki, kd);
                pidController.getGains(1, kp, ki, kd);  // Pitch
                Serial.printf("   Pitch    |  %5.2f  |  %5.2f  |  %5.2f  |\n", kp, ki, kd);
                pidController.getGains(2, kp, ki, kd);  // Yaw
                Serial.printf("   Yaw      |  %5.2f  |  %5.2f  |  %5.2f  |\n", kp, ki, kd);
                
                Serial.println("\n📈 Estado do PID:");
                float roll_int, pitch_int, yaw_int;
                pidController.getIntegrals(roll_int, pitch_int, yaw_int);
                Serial.printf("   Integrais: Roll=%+.4f  Pitch=%+.4f  Yaw=%+.4f\n", 
                             roll_int, pitch_int, yaw_int);
                float roll_err, pitch_err, yaw_err;
                pidController.getErrors(roll_err, pitch_err, yaw_err);
                Serial.printf("   Erros:     Roll=%+.4f  Pitch=%+.4f  Yaw=%+.4f\n", 
                             roll_err, pitch_err, yaw_err);
            }

            Serial.println("========================================\n");
            
            // Mede o tempo que os prints custaram
            last_print_time = micros() - print_start;
            Serial.printf("⏱️  Tempo dos Prints: %lu μs\n", last_print_time);
            Serial.printf("========================================\n");        

            
            prev_ms = micros();
        }
    } else {
        // ===== MODO SERIAL PLOTTER: Printa apenas quando há novo máximo =====
        if (newMaxTime) {
            Serial.print("Atual:");
            Serial.print(loopTime);
            Serial.print(",Media:");
            Serial.print(avgTime, 1);
            Serial.print(",Max:");
            Serial.println(maxTime);
        }
    }
}

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r, float omega_r) {
    // Atualiza a matriz A contínua com as velocidades angulares atuais
    // As três primeiras linhas permanecem constantes

    float alpha_1 = 0.5f;
    float alpha_2 = 0.5f;
    float alpha_3 = 0.5f;      

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
    A[3 * STATE_SIZE + 4] = alpha_1 * ((Iyy - Izz) / Ixx) * r - Ir * omega_r / Ixx;
    A[3 * STATE_SIZE + 5] = (1 - alpha_1) * ((Iyy - Izz) / Ixx) * q;
    
    // Linha 5 (índice 4)
    A[4 * STATE_SIZE + 3] = alpha_2 * ((Izz - Ixx) / Iyy) * r + Ir * omega_r / Iyy;
    A[4 * STATE_SIZE + 5] = (1 - alpha_2) * ((Izz - Ixx) / Iyy) * p;
    
    // Linha 6 (índice 5)
    A[5 * STATE_SIZE + 3] = alpha_3 * ((Ixx - Iyy) / Izz) * q;
    A[5 * STATE_SIZE + 4] = (1 - alpha_3) * ((Ixx - Iyy) / Izz) * p;
    
    // Discretiza a matriz A atualizada
    const float dt = SAMPLING_TIME_S;

    // Calcula A^2 para melhor aproximação da discretização
    float A2[STATE_SIZE * STATE_SIZE] = {0};
    MatrixOperations::matrixMultiply(A, A, A2, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    
    // Ad = I + A*dt + (A^2*dt^2)/2 (aproximação de Taylor de 2ª ordem)
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            if (i == j) {
                Ad[i * STATE_SIZE + j] = 1 + A[i * STATE_SIZE + j] * dt + 
                                       A2[i * STATE_SIZE + j] * dt * dt * 0.5f;
            } else {
                Ad[i * STATE_SIZE + j] = A[i * STATE_SIZE + j] * dt + 
                                       A2[i * STATE_SIZE + j] * dt * dt * 0.5f;
            }
        }
    }

    // Discretiza a matriz B com aproximação de segunda ordem: Bd = (I*dt + A*dt^2/2)*B = B*dt + A*B*dt^2/2
    float AB[STATE_SIZE * CONTROL_SIZE] = {0};
    MatrixOperations::matrixMultiply(A, B, AB, STATE_SIZE, STATE_SIZE, CONTROL_SIZE);

    float dt2_over_2 = dt * dt * 0.5f;

    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            int index = i * CONTROL_SIZE + j;
            Bd[index] = B[index] * dt + AB[index] * dt2_over_2;
        }
    }

    // Atualiza o controlador SDRE com a nova matriz de estados
    // (PID não usa matriz de estados, mas chamamos por compatibilidade)
    if (CONTROLLER_TYPE == 0) {
        sdreController.setStateMatrix(Ad);
    }
}

