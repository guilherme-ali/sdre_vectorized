
#include <AutoLQR.h>
#include <math.h>
#include "MatrixOperations.h"

namespace RiccatiBenchmark {
    const int STATE_SIZE_BENCH = 6;
    const int CONTROL_SIZE_BENCH = 3;
    const int NUM_ITERATIONS = 1000;

    AutoLQR lqr_sda(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_schur(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_vd(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_iter(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);

    unsigned long times_sda[NUM_ITERATIONS];
    unsigned long times_schur[NUM_ITERATIONS];
    unsigned long times_vd[NUM_ITERATIONS];
    unsigned long times_iter[NUM_ITERATIONS];

    float A_bench[STATE_SIZE_BENCH * STATE_SIZE_BENCH];
    float Ad_bench[STATE_SIZE_BENCH * STATE_SIZE_BENCH];
    float Bd_bench[STATE_SIZE_BENCH * CONTROL_SIZE_BENCH];

    const float Ixx = 16.57e-6;
    const float Iyy = 16.57e-6;
    const float Izz = 29.80e-6;
    const float Ir = 1.0e-9;
    const float omega_r = 0;

    float B[STATE_SIZE_BENCH * CONTROL_SIZE_BENCH] = {
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        1/Ixx, 0, 0,
        0, 1/Iyy, 0,
        0, 0, 1/Izz
    };

    float Q[STATE_SIZE_BENCH * STATE_SIZE_BENCH] = {
        100, 0, 0, 0, 0, 0,
        0, 100, 0, 0, 0, 0,
        0, 0, 100, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
    };

    float R[CONTROL_SIZE_BENCH * CONTROL_SIZE_BENCH] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    };

    void updateSystemMatrixBench(float roll, float pitch, float yaw, float p, float q, float r, const float* B_orig) {
        float alpha_1 = 0.5f;
        float alpha_2 = 0.5f;
        float alpha_3 = 0.5f;
        float sin_roll = sin(roll);
        float cos_roll = cos(roll);
        float cos_pitch = cos(pitch);
        float tan_pitch = tan(pitch);
        float inv_cos_pitch = 1.0f / cos_pitch;
        memset(A_bench, 0, sizeof(A_bench));
        A_bench[0 * STATE_SIZE_BENCH + 3] = 1;
        A_bench[0 * STATE_SIZE_BENCH + 4] = sin_roll * tan_pitch;
        A_bench[0 * STATE_SIZE_BENCH + 5] = cos_roll * tan_pitch;
        A_bench[1 * STATE_SIZE_BENCH + 4] = cos_roll;
        A_bench[1 * STATE_SIZE_BENCH + 5] = -sin_roll;
        A_bench[2 * STATE_SIZE_BENCH + 4] = sin_roll * inv_cos_pitch;
        A_bench[2 * STATE_SIZE_BENCH + 5] = cos_roll * inv_cos_pitch;
        A_bench[3 * STATE_SIZE_BENCH + 4] = alpha_1 * ((Iyy - Izz) / Ixx) * r - Ir * omega_r / Ixx;
        A_bench[3 * STATE_SIZE_BENCH + 5] = (1 - alpha_1) * ((Iyy - Izz) / Ixx) * q;
        A_bench[4 * STATE_SIZE_BENCH + 3] = alpha_2 * ((Izz - Ixx) / Iyy) * r - Ir * omega_r / Iyy;
        A_bench[4 * STATE_SIZE_BENCH + 5] = (1 - alpha_2) * ((Izz - Ixx) / Iyy) * p;
        A_bench[5 * STATE_SIZE_BENCH + 3] = alpha_3 * ((Ixx - Iyy) / Izz) * q;
        A_bench[5 * STATE_SIZE_BENCH + 4] = (1 - alpha_3) * ((Ixx - Iyy) / Izz) * p;
        float dt = 0.012f;
        float A2[STATE_SIZE_BENCH * STATE_SIZE_BENCH] = {0};
        MatrixOperations::matrixMultiply(A_bench, A_bench, A2, STATE_SIZE_BENCH, STATE_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < STATE_SIZE_BENCH; i++) {
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                if (i == j) {
                    Ad_bench[i * STATE_SIZE_BENCH + j] = 1 + A_bench[i * STATE_SIZE_BENCH + j] * dt + A2[i * STATE_SIZE_BENCH + j] * dt * dt * 0.5f;
                } else {
                    Ad_bench[i * STATE_SIZE_BENCH + j] = A_bench[i * STATE_SIZE_BENCH + j] * dt + A2[i * STATE_SIZE_BENCH + j] * dt * dt * 0.5f;
                }
            }
        }
        float AB[STATE_SIZE_BENCH * CONTROL_SIZE_BENCH] = {0};
        MatrixOperations::matrixMultiply(A_bench, B_orig, AB, STATE_SIZE_BENCH, STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
        float dt2_over_2 = dt * dt * 0.5f;
        for (int i = 0; i < STATE_SIZE_BENCH; i++) {
            for (int j = 0; j < CONTROL_SIZE_BENCH; j++) {
                int index = i * CONTROL_SIZE_BENCH + j;
                Bd_bench[index] = B_orig[index] * dt + AB[index] * dt2_over_2;
            }
        }
    }

    void run() {
        Serial.begin(115200);
        delay(2000);
        Serial.println("\n\n=========================================================");
        Serial.println("       INICIANDO BENCHMARK DE MÉTODOS RICCATI");
        Serial.println("=========================================================");
        Serial.printf("Iterações: %d\n", NUM_ITERATIONS);
        Serial.println("Aguarde...\n");
        lqr_sda.setCostMatrices(Q, R);
        lqr_schur.setCostMatrices(Q, R);
        lqr_vd.setCostMatrices(Q, R);
        lqr_iter.setCostMatrices(Q, R);
        float roll = 0, pitch = 0, yaw = 0;
        float p = 0, q = 0, r = 0;
        updateSystemMatrixBench(roll, pitch, yaw, p, q, r, B);
        lqr_iter.setStateMatrix(Ad_bench);
        lqr_iter.setInputMatrix(Bd_bench);
        lqr_iter.computeGains("ITERATIVE");
        for (int i = 0; i < NUM_ITERATIONS; i++) {
            roll  += (random(-100, 100) / 10000.0f); 
            pitch += (random(-100, 100) / 10000.0f);
            yaw   += (random(-100, 100) / 10000.0f);
            p     += (random(-100, 100) / 1000.0f);
            q     += (random(-100, 100) / 1000.0f);
            r     += (random(-100, 100) / 1000.0f);
            updateSystemMatrixBench(roll, pitch, yaw, p, q, r, B);
            lqr_sda.setStateMatrix(Ad_bench);
            lqr_sda.setInputMatrix(Bd_bench);
            unsigned long t0 = micros();
            lqr_sda.computeGains("SDA");
            times_sda[i] = micros() - t0;
            lqr_schur.setStateMatrix(Ad_bench);
            lqr_schur.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_schur.computeGains("SCHUR");
            times_schur[i] = micros() - t0;
            lqr_vd.setStateMatrix(Ad_bench);
            lqr_vd.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_vd.computeGains("VAN_DOOREN");
            times_vd[i] = micros() - t0;
            lqr_iter.setStateMatrix(Ad_bench);
            lqr_iter.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_iter.computeGains("ITERATIVE");
            times_iter[i] = micros() - t0;
            delay(5);
        }
        double sum_sda = 0, sum_schur = 0, sum_vd = 0, sum_iter = 0;
        double sq_sum_sda = 0, sq_sum_schur = 0, sq_sum_vd = 0, sq_sum_iter = 0;
        for (int i = 0; i < NUM_ITERATIONS; i++) {
            sum_sda += times_sda[i];
            sq_sum_sda += times_sda[i] * times_sda[i];
            sum_schur += times_schur[i];
            sq_sum_schur += times_schur[i] * times_schur[i];
            sum_vd += times_vd[i];
            sq_sum_vd += times_vd[i] * times_vd[i];
            sum_iter += times_iter[i];
            sq_sum_iter += times_iter[i] * times_iter[i];
        }
        double mean_sda = sum_sda / NUM_ITERATIONS;
        double std_sda = sqrt((sq_sum_sda / NUM_ITERATIONS) - (mean_sda * mean_sda));
        double std_mean_sda = std_sda / sqrt(NUM_ITERATIONS);
        double mean_schur = sum_schur / NUM_ITERATIONS;
        double std_schur = sqrt((sq_sum_schur / NUM_ITERATIONS) - (mean_schur * mean_schur));
        double std_mean_schur = std_schur / sqrt(NUM_ITERATIONS);
        double mean_vd = sum_vd / NUM_ITERATIONS;
        double std_vd = sqrt((sq_sum_vd / NUM_ITERATIONS) - (mean_vd * mean_vd));
        double std_mean_vd = std_vd / sqrt(NUM_ITERATIONS);
        double mean_iter = sum_iter / NUM_ITERATIONS;
        double std_iter = sqrt((sq_sum_iter / NUM_ITERATIONS) - (mean_iter * mean_iter));
        double std_mean_iter = std_iter / sqrt(NUM_ITERATIONS);
        Serial.println("\n================ RESULTADOS DO BENCHMARK ================");
        Serial.printf("%-15s | %-15s | %-15s | %-15s\n", "MÉTODO", "MÉDIA (us)", "DESVIO PAD (us)", "DESVIO PAD MÉDIA");
        Serial.println("--------------------------------------------------------------------------");
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA", mean_sda, std_sda, std_mean_sda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SCHUR", mean_schur, std_schur, std_mean_schur);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "VAN DOOREN", mean_vd, std_vd, std_mean_vd);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ITERATIVE", mean_iter, std_iter, std_mean_iter);
        Serial.println("=========================================================");
        Serial.println("\nReinicie o ESP32 para rodar novamente ou desative o modo benchmark.");
        while(1) { delay(1000); }
    }
}

void setup() {
    RiccatiBenchmark::run();
}

void loop() {}

