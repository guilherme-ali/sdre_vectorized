#ifndef UTILS_H
#define UTILS_H

#include "sensor_config.h"
#include "MPU9250.h"
#include <Adafruit_BMP280.h>

#ifdef USE_MPU6050
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
#endif

// Funções de inicialização
void start_IMU_MPU9250(MPU9250& IMU);
#ifdef USE_MPU6050
    void start_IMU_MPU6050(Adafruit_MPU6050& mpu);
    void calibrate_MPU6050(Adafruit_MPU6050& mpu, float& accel_offset_x, float& accel_offset_y, float& accel_offset_z,
                           float& gyro_offset_x, float& gyro_offset_y, float& gyro_offset_z);
#endif
void start_BMP(Adafruit_BMP280& bmp);

// Funções do magnetômetro QMC5883L (I2C1: SDA=GPIO40, SCL=GPIO41)
void start_QMC5883L(TwoWire& wireInstance);
void setQMC5883LCalibration(float offset_x, float offset_y, float offset_z,
                            float scale_x, float scale_y, float scale_z);
void read_QMC5883L(float& mx, float& my, float& mz);

// Funções de leitura de sensores
#ifdef USE_MPU6050
    void read_MPU6050(Adafruit_MPU6050& mpu, float& ax, float& ay, float& az, 
                      float& gx, float& gy, float& gz,
                      float accel_offset_x, float accel_offset_y, float accel_offset_z,
                      float gyro_offset_x, float gyro_offset_y, float gyro_offset_z);
#endif
void read_MPU9250(MPU9250& IMU, float& ax, float& ay, float& az, 
                  float& gx, float& gy, float& gz, float& mx, float& my, float& mz);

// Funções de cálculo de controle
void calculateMotorOmegaSq(float thrust_signal, float u_torques[], float b_coeff, float d_coeff,
                           float& w1_sq, float& w2_sq, float& w3_sq, float& w4_sq);

// Funções de display
void displayGains();
void displayIMU(float ax, float ay, float az, float gx, float gy, float gz, 
                float mx, float my, float mz, float temp);
void displayBMP(Adafruit_BMP280& bmp);
void displayStates(float states[]);
void displayControlSignals(float u_signal[], float thrust_signal);
void displayMotorOmegaSq(float thrust_signal, float u_torques[], float b_coeff, float d_coeff);
void displayMotorOmegaSqDetailed(float w1_sq, float w2_sq, float w3_sq, float w4_sq);

#endif