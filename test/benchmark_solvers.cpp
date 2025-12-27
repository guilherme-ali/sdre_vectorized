
#include <AutoLQR.h>
#include <math.h>
#include "MatrixOperations.h"

namespace RiccatiBenchmark {
    const int STATE_SIZE_BENCH = 6;
    const int CONTROL_SIZE_BENCH = 3;
    const int NUM_ITERATIONS = 100;

    AutoLQR lqr_sda(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_sda_ss(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_asda(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_sda_scaled(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_adda(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_schur(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_vd(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_iter(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);

    unsigned long times_sda[NUM_ITERATIONS];
    unsigned long times_sda_ss[NUM_ITERATIONS];
    unsigned long times_asda[NUM_ITERATIONS];
    unsigned long times_sda_scaled[NUM_ITERATIONS];
    unsigned long times_adda[NUM_ITERATIONS];
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
        Serial.println("     Incluindo métodos SDA melhorados (SDA-ss, ASDA, Scaled, ADDA)");
        Serial.println("=========================================================");
        Serial.printf("Iterações: %d\n", NUM_ITERATIONS);
        Serial.println("Aguarde...\n");
        lqr_sda.setCostMatrices(Q, R);
        lqr_sda_ss.setCostMatrices(Q, R);
        lqr_asda.setCostMatrices(Q, R);
        lqr_sda_scaled.setCostMatrices(Q, R);
        lqr_adda.setCostMatrices(Q, R);
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
            
            // SDA Original
            lqr_sda.setStateMatrix(Ad_bench);
            lqr_sda.setInputMatrix(Bd_bench);
            unsigned long t0 = micros();
            lqr_sda.computeGains("SDA");
            times_sda[i] = micros() - t0;
            
            // SDA com Single Shift (SDA-ss)
            lqr_sda_ss.setStateMatrix(Ad_bench);
            lqr_sda_ss.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_sda_ss.computeGains("SDA_SS");
            times_sda_ss[i] = micros() - t0;
            
            // ASDA (Adaptive SDA)
            lqr_asda.setStateMatrix(Ad_bench);
            lqr_asda.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_asda.computeGains("ASDA");
            times_asda[i] = micros() - t0;
            
            // SDA Scaled
            lqr_sda_scaled.setStateMatrix(Ad_bench);
            lqr_sda_scaled.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_sda_scaled.computeGains("SDA_SCALED");
            times_sda_scaled[i] = micros() - t0;
            
            // ADDA (Alternating-Directional Doubling Algorithm)
            lqr_adda.setStateMatrix(Ad_bench);
            lqr_adda.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_adda.computeGains("ADDA");
            times_adda[i] = micros() - t0;
            
            // SCHUR
            lqr_schur.setStateMatrix(Ad_bench);
            lqr_schur.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_schur.computeGains("SCHUR");
            times_schur[i] = micros() - t0;
            
            // VAN_DOOREN
            lqr_vd.setStateMatrix(Ad_bench);
            lqr_vd.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_vd.computeGains("VAN_DOOREN");
            times_vd[i] = micros() - t0;
            
            // ITERATIVE
            lqr_iter.setStateMatrix(Ad_bench);
            lqr_iter.setInputMatrix(Bd_bench);
            t0 = micros();
            lqr_iter.computeGains("ITERATIVE");
            times_iter[i] = micros() - t0;
            delay(5);
        }
        
        // Cálculo estatístico para todos os métodos
        double sum_sda = 0, sum_sda_ss = 0, sum_asda = 0, sum_sda_scaled = 0, sum_adda = 0;
        double sum_schur = 0, sum_vd = 0, sum_iter = 0;
        double sq_sum_sda = 0, sq_sum_sda_ss = 0, sq_sum_asda = 0, sq_sum_sda_scaled = 0, sq_sum_adda = 0;
        double sq_sum_schur = 0, sq_sum_vd = 0, sq_sum_iter = 0;
        
        for (int i = 0; i < NUM_ITERATIONS; i++) {
            sum_sda += times_sda[i];
            sq_sum_sda += times_sda[i] * times_sda[i];
            
            sum_sda_ss += times_sda_ss[i];
            sq_sum_sda_ss += times_sda_ss[i] * times_sda_ss[i];
            
            sum_asda += times_asda[i];
            sq_sum_asda += times_asda[i] * times_asda[i];
            
            sum_sda_scaled += times_sda_scaled[i];
            sq_sum_sda_scaled += times_sda_scaled[i] * times_sda_scaled[i];
            
            sum_adda += times_adda[i];
            sq_sum_adda += times_adda[i] * times_adda[i];
            
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
        
        double mean_sda_ss = sum_sda_ss / NUM_ITERATIONS;
        double std_sda_ss = sqrt((sq_sum_sda_ss / NUM_ITERATIONS) - (mean_sda_ss * mean_sda_ss));
        double std_mean_sda_ss = std_sda_ss / sqrt(NUM_ITERATIONS);
        
        double mean_asda = sum_asda / NUM_ITERATIONS;
        double std_asda = sqrt((sq_sum_asda / NUM_ITERATIONS) - (mean_asda * mean_asda));
        double std_mean_asda = std_asda / sqrt(NUM_ITERATIONS);
        
        double mean_sda_scaled = sum_sda_scaled / NUM_ITERATIONS;
        double std_sda_scaled = sqrt((sq_sum_sda_scaled / NUM_ITERATIONS) - (mean_sda_scaled * mean_sda_scaled));
        double std_mean_sda_scaled = std_sda_scaled / sqrt(NUM_ITERATIONS);
        
        double mean_adda = sum_adda / NUM_ITERATIONS;
        double std_adda = sqrt((sq_sum_adda / NUM_ITERATIONS) - (mean_adda * mean_adda));
        double std_mean_adda = std_adda / sqrt(NUM_ITERATIONS);
        
        double mean_schur = sum_schur / NUM_ITERATIONS;
        double std_schur = sqrt((sq_sum_schur / NUM_ITERATIONS) - (mean_schur * mean_schur));
        double std_mean_schur = std_schur / sqrt(NUM_ITERATIONS);
        
        double mean_vd = sum_vd / NUM_ITERATIONS;
        double std_vd = sqrt((sq_sum_vd / NUM_ITERATIONS) - (mean_vd * mean_vd));
        double std_mean_vd = std_vd / sqrt(NUM_ITERATIONS);
        
        double mean_iter = sum_iter / NUM_ITERATIONS;
        double std_iter = sqrt((sq_sum_iter / NUM_ITERATIONS) - (mean_iter * mean_iter));
        double std_mean_iter = std_iter / sqrt(NUM_ITERATIONS);
        
        Serial.println("\n==================== RESULTADOS DO BENCHMARK ====================");
        Serial.printf("%-15s | %-15s | %-15s | %-15s\n", "MÉTODO", "MÉDIA (us)", "DESVIO PAD (us)", "DESVIO PAD MÉDIA");
        Serial.println("---------------------------------------------------------------------------");
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA (orig)", mean_sda, std_sda, std_mean_sda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA-ss", mean_sda_ss, std_sda_ss, std_mean_sda_ss);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ASDA", mean_asda, std_asda, std_mean_asda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA Scaled", mean_sda_scaled, std_sda_scaled, std_mean_sda_scaled);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ADDA", mean_adda, std_adda, std_mean_adda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SCHUR", mean_schur, std_schur, std_mean_schur);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "VAN DOOREN", mean_vd, std_vd, std_mean_vd);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ITERATIVE", mean_iter, std_iter, std_mean_iter);
        Serial.println("==================================================================");
        
        // Comparação de precisão dos ganhos K
        Serial.println("\n=============== COMPARAÇÃO DE PRECISÃO DOS GANHOS K ===============");
        Serial.println("Referência: ITERATIVE (com warm start, alta precisão)");
        
        // Obter ganhos de referência (ITERATIVE)
        float K_ref[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        lqr_iter.exportGains(K_ref);
        
        // Comparar com cada método
        float K_sda[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        float K_sda_ss[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        float K_asda[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        float K_sda_scaled[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        float K_adda[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        float K_schur[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        float K_vd[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        
        lqr_sda.exportGains(K_sda);
        lqr_sda_ss.exportGains(K_sda_ss);
        lqr_asda.exportGains(K_asda);
        lqr_sda_scaled.exportGains(K_sda_scaled);
        lqr_adda.exportGains(K_adda);
        lqr_schur.exportGains(K_schur);
        lqr_vd.exportGains(K_vd);
        
        // Calcular erro RMS para cada método
        auto calcRMSError = [&](float* K_test) {
            float sum_sq = 0;
            for (int i = 0; i < CONTROL_SIZE_BENCH * STATE_SIZE_BENCH; i++) {
                float diff = K_test[i] - K_ref[i];
                sum_sq += diff * diff;
            }
            return sqrt(sum_sq / (CONTROL_SIZE_BENCH * STATE_SIZE_BENCH));
        };
        
        Serial.printf("Erro RMS SDA (orig) vs ITER:   %.6e\n", calcRMSError(K_sda));
        Serial.printf("Erro RMS SDA-ss vs ITER:       %.6e\n", calcRMSError(K_sda_ss));
        Serial.printf("Erro RMS ASDA vs ITER:         %.6e\n", calcRMSError(K_asda));
        Serial.printf("Erro RMS SDA Scaled vs ITER:   %.6e\n", calcRMSError(K_sda_scaled));
        Serial.printf("Erro RMS ADDA vs ITER:         %.6e\n", calcRMSError(K_adda));
        Serial.printf("Erro RMS SCHUR vs ITER:        %.6e\n", calcRMSError(K_schur));
        Serial.printf("Erro RMS VAN DOOREN vs ITER:   %.6e\n", calcRMSError(K_vd));
        Serial.println("==================================================================");
        
        // Imprimir matrizes K para SDA, ITERATIVE e ADDA
        Serial.println("\n=============== MATRIZ K - SDA (Original) ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_sda[i * STATE_SIZE_BENCH + j]);
                if (j < STATE_SIZE_BENCH - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        Serial.println("\n=============== MATRIZ K - ITERATIVE ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_ref[i * STATE_SIZE_BENCH + j]);
                if (j < STATE_SIZE_BENCH - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        Serial.println("\n=============== MATRIZ K - ADDA ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_adda[i * STATE_SIZE_BENCH + j]);
                if (j < STATE_SIZE_BENCH - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        Serial.println("\n=============== MATRIZ K - VAN DOOREN ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_vd[i * STATE_SIZE_BENCH + j]);
                if (j < STATE_SIZE_BENCH - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        Serial.println("\nReinicie o ESP32 para rodar novamente ou desative o modo benchmark.");
        while(1) { delay(1000); }
    }
}

void setup() {
    RiccatiBenchmark::run();
}

void loop() {}

