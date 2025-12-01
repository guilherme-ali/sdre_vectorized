#include "utils.h"
#include <AutoLQR.h>
#include <Wire.h>

#define STATE_SIZE 6
#define CONTROL_SIZE 3

// Declaração externa do controlador (definido na main.cpp)
extern AutoLQR controller;

// ============= FUNÇÕES DE INICIALIZAÇÃO =============

void start_IMU_MPU9250(MPU9250& IMU) {
    Wire.begin(); // Inicializa a comunicação I2C
    int status_IMU = IMU.begin();
    if (status_IMU < 0) {
        Serial.println("Falha na inicialização do MPU9250");
        Serial.print("Status_IMU: ");
        Serial.println(status_IMU);
        if(status_IMU != -5){
            while(1) {}
        }
    }

    Serial.println("MPU9250 inicializado com sucesso!");

    // Define o range do acelerômetro para +/-2G 
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // Define o range do giroscópio para +/-250 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    // Define a largura de banda do DLPF para 92 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);
    // Define o divisor de taxa de amostragem para 9 (100 Hz)
    IMU.setSrd(9);

    /*
    Serial.println("Calibrando acelerômetro");
    IMU.calibrateAccel();
    Serial.println("Calibrando giroscópio");
    IMU.calibrateGyro();
    Serial.println("Calibrando magnetômetro");
    IMU.calibrateMag();
    */
}

#ifdef USE_MPU6050
void start_IMU_MPU6050(Adafruit_MPU6050& mpu) {
    // Inicializa I2C com os pinos corretos para ESP32-S2
    Wire.begin(11, 10); // SDA = GPIO11, SCL = GPIO10

    Wire.setClock(1000000); // Fast Mode (padrão é 100kHz)
    
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("Falha ao inicializar MPU6050!");
        while (1) {
            delay(10);
        }
    }
    
    Serial.println("MPU6050 inicializado com sucesso!");
    
    // Configuração do MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    
    delay(100); // Aguarda estabilização
}

void calibrate_MPU6050(Adafruit_MPU6050& mpu, float& accel_offset_x, float& accel_offset_y, float& accel_offset_z,
                       float& gyro_offset_x, float& gyro_offset_y, float& gyro_offset_z) {
    Serial.println("=========================================");
    Serial.println("Iniciando calibração do MPU6050...");
    Serial.println("IMPORTANTE: Deixe o sensor COMPLETAMENTE IMÓVEL!");
    Serial.println("=========================================");
    
    const int numSamples = 1000; // Número de amostras para calibração
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    
    // Aguarda 3 segundos antes de iniciar
    for(int i = 3; i > 0; i--) {
        Serial.print("Iniciando em ");
        Serial.print(i);
        Serial.println(" segundos...");
        delay(1000);
    }
    
    Serial.println("Coletando dados...");
    
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        accel_sum_x += a.acceleration.x;
        accel_sum_y += a.acceleration.y;
        accel_sum_z += a.acceleration.z;
        
        gyro_sum_x += g.gyro.x;
        gyro_sum_y += g.gyro.y;
        gyro_sum_z += g.gyro.z;
        
        if (i % 100 == 0) {
            Serial.print("Progresso: ");
            Serial.print((i * 100) / numSamples);
            Serial.println("%");
        }
        
        delay(2); // Pequeno delay entre leituras
    }
    
    // Calcula as médias
    accel_offset_x = accel_sum_x / numSamples;
    accel_offset_y = accel_sum_y / numSamples;
    accel_offset_z = (accel_sum_z / numSamples) - 9.81; // Subtrai gravidade (sensor está na horizontal, Z aponta para cima)
    
    gyro_offset_x = gyro_sum_x / numSamples;
    gyro_offset_y = gyro_sum_y / numSamples;
    gyro_offset_z = gyro_sum_z / numSamples;
    
    Serial.println("=========================================");
    Serial.println("Calibração concluída!");
    Serial.println("Offsets calculados:");
    Serial.print("Accel X: "); Serial.print(accel_offset_x, 6); Serial.println(" m/s²");
    Serial.print("Accel Y: "); Serial.print(accel_offset_y, 6); Serial.println(" m/s²");
    Serial.print("Accel Z: "); Serial.print(accel_offset_z, 6); Serial.println(" m/s²");
    Serial.print("Gyro X: "); Serial.print(gyro_offset_x, 6); Serial.println(" rad/s");
    Serial.print("Gyro Y: "); Serial.print(gyro_offset_y, 6); Serial.println(" rad/s");
    Serial.print("Gyro Z: "); Serial.print(gyro_offset_z, 6); Serial.println(" rad/s");
    Serial.println("=========================================");
}

void read_MPU6050(Adafruit_MPU6050& mpu, float& ax, float& ay, float& az, 
                  float& gx, float& gy, float& gz,
                  float accel_offset_x, float accel_offset_y, float accel_offset_z,
                  float gyro_offset_x, float gyro_offset_y, float gyro_offset_z) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Aplica os offsets de calibração e converte para as unidades corretas
    ax = (a.acceleration.x - accel_offset_x) / 9.81; // m/s² para g
    ay = (a.acceleration.y - accel_offset_y) / 9.81;
    az = (a.acceleration.z - accel_offset_z) / 9.81; // REMOVI O SINAL NEGATIVO

    gx = g.gyro.x - gyro_offset_x; // Já em rad/s
    gy = g.gyro.y - gyro_offset_y;
    gz = g.gyro.z - gyro_offset_z;
}
#endif

void start_BMP(Adafruit_BMP280& bmp) {
    unsigned status_bmp = bmp.begin(0x76);
    if (!status_bmp) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

    Serial.println("BMP280 inicializado com sucesso!");

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                Adafruit_BMP280::SAMPLING_X2,
                Adafruit_BMP280::SAMPLING_X16,
                Adafruit_BMP280::FILTER_X16,
                Adafruit_BMP280::STANDBY_MS_1);
}

// ============= FUNÇÕES DE LEITURA =============

void read_MPU9250(MPU9250& IMU, float& ax, float& ay, float& az, 
                  float& gx, float& gy, float& gz, float& mx, float& my, float& mz) {
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
}

// ============= FUNÇÕES DE CÁLCULO DE CONTROLE =============

void calculateMotorOmegaSq(float thrust_signal, float u_torques[], float b_coeff, float d_coeff,
                           float& w1_sq, float& w2_sq, float& w3_sq, float& w4_sq) {
    // Verifica se os coeficientes são válidos
    if (b_coeff == 0.0f || d_coeff == 0.0f) {
        Serial.println("ERRO: Coeficientes b ou d não podem ser zero!");
        w1_sq = w2_sq = w3_sq = w4_sq = 0;
        return;
    }

    // Pré-calcula os inversos para otimização
    float inv_4b = 1.0f / (4.0f * b_coeff);
    float inv_2b = 1.0f / (2.0f * b_coeff);
    float inv_4d = 1.0f / (4.0f * d_coeff);

    // Extrai os sinais de controle
    float u1 = thrust_signal;    // Empuxo total
    float u2 = u_torques[0];     // Torque de Rolagem (Roll)
    float u3 = u_torques[1];     // Torque de Arfagem (Pitch)
    float u4 = u_torques[2];     // Torque de Guinada (Yaw)

    // Matriz de alocação de controle conforme a imagem:
    // [ω1²]   [1/(4b)  -1/(2b)   1/(2b)  -1/(4d)] [u1]
    // [ω2²] = [1/(4b)  -1/(2b)  -1/(2b)   1/(4d)] [u2]
    // [ω3²]   [1/(4b)   1/(2b)  -1/(2b)  -1/(4d)] [u3]
    // [ω4²]   [1/(4b)   1/(2b)   1/(2b)   1/(4d)] [u4]
    
    w1_sq = u1 * inv_4b - u2 * inv_2b + u3 * inv_2b - u4 * inv_4d;  // Motor 1
    w2_sq = u1 * inv_4b - u2 * inv_2b - u3 * inv_2b + u4 * inv_4d;  // Motor 2
    w3_sq = u1 * inv_4b + u2 * inv_2b - u3 * inv_2b - u4 * inv_4d;  // Motor 3
    w4_sq = u1 * inv_4b + u2 * inv_2b + u3 * inv_2b + u4 * inv_4d;  // Motor 4

    // Garante que não há valores negativos (motores não podem girar ao contrário)
    if (w1_sq < 0) w1_sq = 0;
    if (w2_sq < 0) w2_sq = 0;
    if (w3_sq < 0) w3_sq = 0;
    if (w4_sq < 0) w4_sq = 0;
}

// ============= FUNÇÕES DE DISPLAY =============

void displayGains(){
    float exportedGains[CONTROL_SIZE * STATE_SIZE];
    controller.exportGains(exportedGains);

    float exportedKr[CONTROL_SIZE * CONTROL_SIZE];
    controller.exportKr(exportedKr);
    Serial.println("Ganhos do LQR calculados com sucesso");

    Serial.println("Ganhos Exportados (K):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            Serial.print(exportedGains[i * STATE_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    Serial.println("Matriz Kr (ganho de referência):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Serial.print(exportedKr[i * CONTROL_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void displayIMU(float ax, float ay, float az, float gx, float gy, float gz, 
                float mx, float my, float mz, float temp) {
    Serial.print(ax, 6);
    Serial.print("\t");
    Serial.print(ay, 6);
    Serial.print("\t");
    Serial.print(az, 6);
    Serial.print("\t");
    Serial.print(gx, 6);
    Serial.print("\t");
    Serial.print(gy, 6);
    Serial.print("\t");
    Serial.print(gz, 6);
    Serial.print("\t");
    Serial.print(mx, 6);
    Serial.print("\t");
    Serial.print(my, 6);
    Serial.print("\t");
    Serial.print(mz, 6);
    Serial.print("\t");
    Serial.println(temp, 6);
}

void displayBMP(Adafruit_BMP280& bmp) {
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(1013.25);
    
    Serial.print("Temp_BMP:"); Serial.print(temperature);
    Serial.print("°C,Press:"); Serial.print(pressure/100.0);
    Serial.print("hPa,Alt:"); Serial.print(altitude);
    Serial.println("m");
}

void displayStates(float states[]) { 
    Serial.print("Roll:"); Serial.print(states[0]);
    Serial.print(",Pitch:"); Serial.print(states[1]);
    Serial.print(",Yaw:"); Serial.print(states[2]);
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
    float w1_sq, w2_sq, w3_sq, w4_sq;
    
    // Usa a função de cálculo
    calculateMotorOmegaSq(thrust_signal, u_torques, b_coeff, d_coeff,
                          w1_sq, w2_sq, w3_sq, w4_sq);

    displayMotorOmegaSqDetailed(w1_sq, w2_sq, w3_sq, w4_sq);
}

void displayMotorOmegaSqDetailed(float w1_sq, float w2_sq, float w3_sq, float w4_sq) {
    Serial.print("GPIO3(ω²): "); Serial.print(w1_sq, 2);
    Serial.print(" | GPIO4(ω²): "); Serial.print(w2_sq, 2);
    Serial.print(" | GPIO5(ω²): "); Serial.print(w3_sq, 2);
    Serial.print(" | GPIO6(ω²): "); Serial.println(w4_sq, 2);
}