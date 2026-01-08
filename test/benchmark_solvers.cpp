
#include <AutoLQR.h>
#include <math.h>
#include <utility>
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
    AutoLQR lqr_vd(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);
    AutoLQR lqr_iter(STATE_SIZE_BENCH, CONTROL_SIZE_BENCH);

    unsigned long times_sda[NUM_ITERATIONS];
    unsigned long times_sda_ss[NUM_ITERATIONS];
    unsigned long times_asda[NUM_ITERATIONS];
    unsigned long times_sda_scaled[NUM_ITERATIONS];
    unsigned long times_adda[NUM_ITERATIONS];
    unsigned long times_vd[NUM_ITERATIONS];
    unsigned long times_iter[NUM_ITERATIONS];

    // Arrays para armazenar iterações de convergência
    int iters_sda[NUM_ITERATIONS];
    int iters_sda_ss[NUM_ITERATIONS];
    int iters_asda[NUM_ITERATIONS];
    int iters_sda_scaled[NUM_ITERATIONS];
    int iters_adda[NUM_ITERATIONS];
    int iters_vd[NUM_ITERATIONS];
    int iters_iter[NUM_ITERATIONS];

    // Arrays para armazenar resíduos finais
    float residuals_sda[NUM_ITERATIONS];
    float residuals_sda_ss[NUM_ITERATIONS];
    float residuals_asda[NUM_ITERATIONS];
    float residuals_sda_scaled[NUM_ITERATIONS];
    float residuals_adda[NUM_ITERATIONS];
    float residuals_vd[NUM_ITERATIONS];
    float residuals_iter[NUM_ITERATIONS];

    // Arrays para armazenar uso de memória (heap antes - heap depois)
    int mem_sda[NUM_ITERATIONS];
    int mem_sda_ss[NUM_ITERATIONS];
    int mem_asda[NUM_ITERATIONS];
    int mem_sda_scaled[NUM_ITERATIONS];
    int mem_adda[NUM_ITERATIONS];
    int mem_vd[NUM_ITERATIONS];
    int mem_iter[NUM_ITERATIONS];

    // Matrizes para histórico de resíduos (10 primeiras iterações) - última execução
    float res_history_sda[10];
    float res_history_sda_ss[10];
    float res_history_asda[10];
    float res_history_sda_scaled[10];
    float res_history_adda[10];
    float res_history_vd[10];
    float res_history_iter[10];

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
            
            size_t heap_before, heap_after;
            
            // SDA Original
            lqr_sda.setStateMatrix(Ad_bench);
            lqr_sda.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            unsigned long t0 = micros();
            lqr_sda.computeGains("SDA");
            times_sda[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_sda[i] = heap_before - heap_after;
            iters_sda[i] = lqr_sda.getLastIterations();
            residuals_sda[i] = lqr_sda.getLastResidual();
            
            // SDA com Single Shift (SDA-ss)
            lqr_sda_ss.setStateMatrix(Ad_bench);
            lqr_sda_ss.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            t0 = micros();
            lqr_sda_ss.computeGains("SDA_SS");
            times_sda_ss[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_sda_ss[i] = heap_before - heap_after;
            iters_sda_ss[i] = lqr_sda_ss.getLastIterations();
            residuals_sda_ss[i] = lqr_sda_ss.getLastResidual();
            
            // ASDA (Adaptive SDA)
            lqr_asda.setStateMatrix(Ad_bench);
            lqr_asda.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            t0 = micros();
            lqr_asda.computeGains("ASDA");
            times_asda[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_asda[i] = heap_before - heap_after;
            iters_asda[i] = lqr_asda.getLastIterations();
            residuals_asda[i] = lqr_asda.getLastResidual();
            
            // SDA Scaled
            lqr_sda_scaled.setStateMatrix(Ad_bench);
            lqr_sda_scaled.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            t0 = micros();
            lqr_sda_scaled.computeGains("SDA_SCALED");
            times_sda_scaled[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_sda_scaled[i] = heap_before - heap_after;
            iters_sda_scaled[i] = lqr_sda_scaled.getLastIterations();
            residuals_sda_scaled[i] = lqr_sda_scaled.getLastResidual();
            
            // ADDA (Alternating-Directional Doubling Algorithm)
            lqr_adda.setStateMatrix(Ad_bench);
            lqr_adda.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            t0 = micros();
            lqr_adda.computeGains("ADDA");
            times_adda[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_adda[i] = heap_before - heap_after;
            iters_adda[i] = lqr_adda.getLastIterations();
            residuals_adda[i] = lqr_adda.getLastResidual();
            
            // VAN_DOOREN
            lqr_vd.setStateMatrix(Ad_bench);
            lqr_vd.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            t0 = micros();
            lqr_vd.computeGains("VAN_DOOREN");
            times_vd[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_vd[i] = heap_before - heap_after;
            iters_vd[i] = lqr_vd.getLastIterations();
            residuals_vd[i] = lqr_vd.getLastResidual();
            
            // ITERATIVE
            lqr_iter.setStateMatrix(Ad_bench);
            lqr_iter.setInputMatrix(Bd_bench);
            heap_before = ESP.getFreeHeap();
            t0 = micros();
            lqr_iter.computeGains("ITERATIVE");
            times_iter[i] = micros() - t0;
            heap_after = ESP.getFreeHeap();
            mem_iter[i] = heap_before - heap_after;
            iters_iter[i] = lqr_iter.getLastIterations();
            residuals_iter[i] = lqr_iter.getLastResidual();
            delay(5);
        }
        
        // Coletar histórico de resíduos da última execução
        lqr_sda.getResidualHistory(res_history_sda);
        lqr_sda_ss.getResidualHistory(res_history_sda_ss);
        lqr_asda.getResidualHistory(res_history_asda);
        lqr_sda_scaled.getResidualHistory(res_history_sda_scaled);
        lqr_adda.getResidualHistory(res_history_adda);
        lqr_vd.getResidualHistory(res_history_vd);
        lqr_iter.getResidualHistory(res_history_iter);
        
        // Cálculo estatístico para todos os métodos
        double sum_sda = 0, sum_sda_ss = 0, sum_asda = 0, sum_sda_scaled = 0, sum_adda = 0;
        double sum_vd = 0, sum_iter = 0;
        double sq_sum_sda = 0, sq_sum_sda_ss = 0, sq_sum_asda = 0, sq_sum_sda_scaled = 0, sq_sum_adda = 0;
        double sq_sum_vd = 0, sq_sum_iter = 0;
        
        for (int i = 0; i < NUM_ITERATIONS; i++) {
            double t;
            
            t = (double)times_sda[i];
            sum_sda += t;
            sq_sum_sda += t * t;
            
            t = (double)times_sda_ss[i];
            sum_sda_ss += t;
            sq_sum_sda_ss += t * t;
            
            t = (double)times_asda[i];
            sum_asda += t;
            sq_sum_asda += t * t;
            
            t = (double)times_sda_scaled[i];
            sum_sda_scaled += t;
            sq_sum_sda_scaled += t * t;
            
            t = (double)times_adda[i];
            sum_adda += t;
            sq_sum_adda += t * t;
            
            t = (double)times_vd[i];
            sum_vd += t;
            sq_sum_vd += t * t;
            
            t = (double)times_iter[i];
            sum_iter += t;
            sq_sum_iter += t * t;
        }
        
        // Função auxiliar para calcular desvio padrão de forma segura
        auto safe_std = [](double sq_sum, double sum, int n) -> double {
            double variance = (sq_sum / n) - (sum / n) * (sum / n);
            return (variance > 0) ? sqrt(variance) : 0.0;
        };
        
        double mean_sda = sum_sda / NUM_ITERATIONS;
        double std_sda = safe_std(sq_sum_sda, sum_sda, NUM_ITERATIONS);
        double std_mean_sda = std_sda / sqrt(NUM_ITERATIONS);
        
        double mean_sda_ss = sum_sda_ss / NUM_ITERATIONS;
        double std_sda_ss = safe_std(sq_sum_sda_ss, sum_sda_ss, NUM_ITERATIONS);
        double std_mean_sda_ss = std_sda_ss / sqrt(NUM_ITERATIONS);
        
        double mean_asda = sum_asda / NUM_ITERATIONS;
        double std_asda = safe_std(sq_sum_asda, sum_asda, NUM_ITERATIONS);
        double std_mean_asda = std_asda / sqrt(NUM_ITERATIONS);
        
        double mean_sda_scaled = sum_sda_scaled / NUM_ITERATIONS;
        double std_sda_scaled = safe_std(sq_sum_sda_scaled, sum_sda_scaled, NUM_ITERATIONS);
        double std_mean_sda_scaled = std_sda_scaled / sqrt(NUM_ITERATIONS);
        
        double mean_adda = sum_adda / NUM_ITERATIONS;
        double std_adda = safe_std(sq_sum_adda, sum_adda, NUM_ITERATIONS);
        double std_mean_adda = std_adda / sqrt(NUM_ITERATIONS);
        
        double mean_vd = sum_vd / NUM_ITERATIONS;
        double std_vd = safe_std(sq_sum_vd, sum_vd, NUM_ITERATIONS);
        double std_mean_vd = std_vd / sqrt(NUM_ITERATIONS);
        
        double mean_iter = sum_iter / NUM_ITERATIONS;
        double std_iter = safe_std(sq_sum_iter, sum_iter, NUM_ITERATIONS);
        double std_mean_iter = std_iter / sqrt(NUM_ITERATIONS);
        
        Serial.println("\n==================== RESULTADOS DO BENCHMARK ====================");
        Serial.printf("%-15s | %-15s | %-15s | %-15s\n", "MÉTODO", "MÉDIA (us)", "DESVIO PAD (us)", "DESVIO PAD MÉDIA");
        Serial.println("---------------------------------------------------------------------------");
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA (orig)", mean_sda, std_sda, std_mean_sda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA-ss", mean_sda_ss, std_sda_ss, std_mean_sda_ss);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ASDA", mean_asda, std_asda, std_mean_asda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "SDA Scaled", mean_sda_scaled, std_sda_scaled, std_mean_sda_scaled);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ADDA", mean_adda, std_adda, std_mean_adda);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "VAN DOOREN", mean_vd, std_vd, std_mean_vd);
        Serial.printf("%-15s | %-15.2f | %-15.2f | %-15.4f\n", "ITERATIVE", mean_iter, std_iter, std_mean_iter);
        Serial.println("==================================================================");
        
        // ================================================================
        // TAXA DE CONVERGÊNCIA DOS RESÍDUOS
        // ================================================================
        Serial.println("\n============= TAXA DE CONVERGÊNCIA DOS RESÍDUOS =================");
        
        // Calcular média de iterações e resíduos para cada método
        auto calcIterStats = [](int* iters, int n) -> std::pair<double, double> {
            double sum = 0, sq_sum = 0;
            for (int i = 0; i < n; i++) {
                sum += iters[i];
                sq_sum += (double)iters[i] * iters[i];
            }
            double mean = sum / n;
            double variance = (sq_sum / n) - mean * mean;
            double std = (variance > 0) ? sqrt(variance) : 0;
            return {mean, std};
        };
        
        auto calcResidualStats = [](float* residuals, int n) -> std::pair<double, double> {
            double sum = 0, sq_sum = 0;
            for (int i = 0; i < n; i++) {
                sum += residuals[i];
                sq_sum += (double)residuals[i] * residuals[i];
            }
            double mean = sum / n;
            double variance = (sq_sum / n) - mean * mean;
            double std = (variance > 0) ? sqrt(variance) : 0;
            return {mean, std};
        };
        
        auto [mean_iters_sda, std_iters_sda] = calcIterStats(iters_sda, NUM_ITERATIONS);
        auto [mean_iters_sda_ss, std_iters_sda_ss] = calcIterStats(iters_sda_ss, NUM_ITERATIONS);
        auto [mean_iters_asda, std_iters_asda] = calcIterStats(iters_asda, NUM_ITERATIONS);
        auto [mean_iters_sda_scaled, std_iters_sda_scaled] = calcIterStats(iters_sda_scaled, NUM_ITERATIONS);
        auto [mean_iters_adda, std_iters_adda] = calcIterStats(iters_adda, NUM_ITERATIONS);
        auto [mean_iters_vd, std_iters_vd] = calcIterStats(iters_vd, NUM_ITERATIONS);
        auto [mean_iters_iter, std_iters_iter] = calcIterStats(iters_iter, NUM_ITERATIONS);
        
        auto [mean_res_sda, std_res_sda] = calcResidualStats(residuals_sda, NUM_ITERATIONS);
        auto [mean_res_sda_ss, std_res_sda_ss] = calcResidualStats(residuals_sda_ss, NUM_ITERATIONS);
        auto [mean_res_asda, std_res_asda] = calcResidualStats(residuals_asda, NUM_ITERATIONS);
        auto [mean_res_sda_scaled, std_res_sda_scaled] = calcResidualStats(residuals_sda_scaled, NUM_ITERATIONS);
        auto [mean_res_adda, std_res_adda] = calcResidualStats(residuals_adda, NUM_ITERATIONS);
        auto [mean_res_vd, std_res_vd] = calcResidualStats(residuals_vd, NUM_ITERATIONS);
        auto [mean_res_iter, std_res_iter] = calcResidualStats(residuals_iter, NUM_ITERATIONS);
        
        Serial.printf("%-15s | %-12s | %-12s | %-15s | %-15s\n", 
                      "MÉTODO", "ITER MÉDIA", "ITER STD", "RESÍDUO MÉDIO", "RESÍDUO STD");
        Serial.println("---------------------------------------------------------------------------");
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "SDA (orig)", mean_iters_sda, std_iters_sda, mean_res_sda, std_res_sda);
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "SDA-ss", mean_iters_sda_ss, std_iters_sda_ss, mean_res_sda_ss, std_res_sda_ss);
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "ASDA", mean_iters_asda, std_iters_asda, mean_res_asda, std_res_asda);
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "SDA Scaled", mean_iters_sda_scaled, std_iters_sda_scaled, mean_res_sda_scaled, std_res_sda_scaled);
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "ADDA", mean_iters_adda, std_iters_adda, mean_res_adda, std_res_adda);
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "VAN DOOREN", mean_iters_vd, std_iters_vd, mean_res_vd, std_res_vd);
        Serial.printf("%-15s | %-12.2f | %-12.2f | %-15.2e | %-15.2e\n", "ITERATIVE", mean_iters_iter, std_iters_iter, mean_res_iter, std_res_iter);
        Serial.println("==================================================================");
        
        // ================================================================
        // HISTÓRICO DE RESÍDUOS (10 PRIMEIRAS ITERAÇÕES)
        // ================================================================
        Serial.println("\n======= EVOLUÇÃO DOS RESÍDUOS (10 PRIMEIRAS ITERAÇÕES) =======");
        Serial.printf("%-12s", "ITER");
        for (int j = 1; j <= 10; j++) {
            Serial.printf(" | %-10d", j);
        }
        Serial.println();
        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
        
        auto printResHistory = [](const char* name, float* hist) {
            Serial.printf("%-12s", name);
            for (int j = 0; j < 10; j++) {
                if (hist[j] > 0) {
                    Serial.printf(" | %-10.2e", hist[j]);
                } else {
                    Serial.printf(" | %-10s", "0");
                }
            }
            Serial.println();
        };
        
        printResHistory("SDA (orig)", res_history_sda);
        printResHistory("SDA-ss", res_history_sda_ss);
        printResHistory("ASDA", res_history_asda);
        printResHistory("SDA Scaled", res_history_sda_scaled);
        printResHistory("ADDA", res_history_adda);
        printResHistory("VAN DOOREN", res_history_vd);
        printResHistory("ITERATIVE", res_history_iter);
        Serial.println("==================================================================");
        
        // ================================================================
        // USO DE MEMÓRIA POR MÉTODO
        // ================================================================
        Serial.println("\n============== USO DE MEMÓRIA HEAP POR MÉTODO ==================");
        
        auto calcMemStats = [](int* mem, int n) -> std::pair<double, double> {
            double sum = 0, sq_sum = 0;
            for (int i = 0; i < n; i++) {
                sum += mem[i];
                sq_sum += (double)mem[i] * mem[i];
            }
            double mean = sum / n;
            double variance = (sq_sum / n) - mean * mean;
            double std = (variance > 0) ? sqrt(variance) : 0;
            return {mean, std};
        };
        
        auto [mean_mem_sda, std_mem_sda] = calcMemStats(mem_sda, NUM_ITERATIONS);
        auto [mean_mem_sda_ss, std_mem_sda_ss] = calcMemStats(mem_sda_ss, NUM_ITERATIONS);
        auto [mean_mem_asda, std_mem_asda] = calcMemStats(mem_asda, NUM_ITERATIONS);
        auto [mean_mem_sda_scaled, std_mem_sda_scaled] = calcMemStats(mem_sda_scaled, NUM_ITERATIONS);
        auto [mean_mem_adda, std_mem_adda] = calcMemStats(mem_adda, NUM_ITERATIONS);
        auto [mean_mem_vd, std_mem_vd] = calcMemStats(mem_vd, NUM_ITERATIONS);
        auto [mean_mem_iter, std_mem_iter] = calcMemStats(mem_iter, NUM_ITERATIONS);
        
        Serial.printf("%-15s | %-18s | %-18s\n", "MÉTODO", "MEM MÉDIA (bytes)", "MEM STD (bytes)");
        Serial.println("---------------------------------------------------------------------------");
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "SDA (orig)", mean_mem_sda, std_mem_sda);
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "SDA-ss", mean_mem_sda_ss, std_mem_sda_ss);
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "ASDA", mean_mem_asda, std_mem_asda);
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "SDA Scaled", mean_mem_sda_scaled, std_mem_sda_scaled);
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "ADDA", mean_mem_adda, std_mem_adda);
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "VAN DOOREN", mean_mem_vd, std_mem_vd);
        Serial.printf("%-15s | %-18.2f | %-18.2f\n", "ITERATIVE", mean_mem_iter, std_mem_iter);
        Serial.println("==================================================================");
        
        // ================================================================
        // RESUMO GERAL DO SISTEMA
        // ================================================================
        Serial.println("\n================= USO DE MEMÓRIA DO SISTEMA ====================");
        Serial.printf("Heap livre:        %u bytes\n", ESP.getFreeHeap());
        Serial.printf("Heap mínimo:       %u bytes\n", ESP.getMinFreeHeap());
        Serial.printf("Heap total:        %u bytes\n", ESP.getHeapSize());
        Serial.printf("Stack livre (aprox): %u bytes\n", uxTaskGetStackHighWaterMark(NULL) * 4);
        Serial.printf("PSRAM livre:       %u bytes\n", ESP.getFreePsram());
        Serial.printf("PSRAM total:       %u bytes\n", ESP.getPsramSize());
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
        float K_vd[CONTROL_SIZE_BENCH * STATE_SIZE_BENCH];
        
        lqr_sda.exportGains(K_sda);
        lqr_sda_ss.exportGains(K_sda_ss);
        lqr_asda.exportGains(K_asda);
        lqr_sda_scaled.exportGains(K_sda_scaled);
        lqr_adda.exportGains(K_adda);
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
        
        Serial.println("\n=============== MATRIZ K - SDA-SS (Single Shift) ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_sda_ss[i * STATE_SIZE_BENCH + j]);
                if (j < STATE_SIZE_BENCH - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        Serial.println("\n=============== MATRIZ K - ASDA (Adaptive) ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_asda[i * STATE_SIZE_BENCH + j]);
                if (j < STATE_SIZE_BENCH - 1) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        Serial.println("\n=============== MATRIZ K - SDA SCALED ===============");
        Serial.printf("K [%d x %d]:\n", CONTROL_SIZE_BENCH, STATE_SIZE_BENCH);
        for (int i = 0; i < CONTROL_SIZE_BENCH; i++) {
            Serial.print("  [");
            for (int j = 0; j < STATE_SIZE_BENCH; j++) {
                Serial.printf("%12.6f", K_sda_scaled[i * STATE_SIZE_BENCH + j]);
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

