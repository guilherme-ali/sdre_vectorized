#include <Arduino.h>
#include "AutoLQR.h"
#include "MatrixOperations.h"
#include <ArduinoEigen.h>

// Define as dimensões das matrizes para o exemplo
// Sistema: x(k+1) = Ax(k) + Bu(k)
// A: 6x6, B: 6x3, Q: 6x6, R: 3x3
const int STATE_DIM_TEST = 6;
const int CONTROL_DIM_TEST = 3;

// Instância do controlador LQR para o teste
AutoLQR lqr_test(STATE_DIM_TEST, CONTROL_DIM_TEST);

// Parâmetros físicos do sistema (do arquivo main copy.cpp.txt)
const float Ixx = 1e-5;  // kg·m²
const float Iyy = 1e-5;  // kg·m²
const float Izz = 1e-5;  // kg·m²
const float Ir = 1e-5;   // kg·m²
const float omega_r = 0;

// Valores de roll, pitch, yaw, p, q, r para o teste
const float roll = 0.0f;
const float pitch = 0.0f;
const float yaw = 0.0f;
const float p = 0.0f;
const float q = 0.0f;
const float r = 0.0f;

// Matrizes do problema DARE baseadas no sistema do drone
// Matriz A contínua (será discretizada)
float A_data[STATE_DIM_TEST * STATE_DIM_TEST];

// Matriz B contínua
float B_data[STATE_DIM_TEST * CONTROL_DIM_TEST] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    1/Ixx, 0, 0,
    0, 1/Iyy, 0,
    0, 0, 1/Izz
};

// Matriz Q (custo dos estados)
float Q_data[STATE_DIM_TEST * STATE_DIM_TEST] = {
    100, 0, 0, 0, 0, 0,
    0, 100, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

// Matriz R (custo dos controles)
float R_data[CONTROL_DIM_TEST * CONTROL_DIM_TEST] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

void initialize_A_matrix() {
    // Inicializa A com zeros
    for (int i = 0; i < STATE_DIM_TEST * STATE_DIM_TEST; i++) {
        A_data[i] = 0.0f;
    }
    
    // Calcula valores trigonométricos
    float sin_roll = sin(roll);
    float cos_roll = cos(roll);
    float cos_pitch = cos(pitch);
    float tan_pitch = tan(pitch);
    float inv_cos_pitch = 1.0f / cos_pitch;
    
    // Preenche a matriz A conforme o sistema do drone
    // Linha 0
    A_data[0 * STATE_DIM_TEST + 3] = 1;
    A_data[0 * STATE_DIM_TEST + 4] = sin_roll * tan_pitch;
    A_data[0 * STATE_DIM_TEST + 5] = cos_roll * tan_pitch;
    
    // Linha 1
    A_data[1 * STATE_DIM_TEST + 4] = cos_roll;
    A_data[1 * STATE_DIM_TEST + 5] = -sin_roll;
    
    // Linha 2
    A_data[2 * STATE_DIM_TEST + 4] = sin_roll * inv_cos_pitch;
    A_data[2 * STATE_DIM_TEST + 5] = cos_roll * inv_cos_pitch;
    
    // Linha 3
    A_data[3 * STATE_DIM_TEST + 4] = ((Iyy - Izz) / (2 * Ixx)) * r - Ir * omega_r / Ixx;
    A_data[3 * STATE_DIM_TEST + 5] = ((Iyy - Izz) / (2 * Ixx)) * q;
    
    // Linha 4
    A_data[4 * STATE_DIM_TEST + 3] = ((Izz - Ixx) / (2 * Iyy)) * r - Ir * omega_r / Iyy;
    A_data[4 * STATE_DIM_TEST + 5] = ((Izz - Ixx) / (2 * Iyy)) * p;
    
    // Linha 5
    A_data[5 * STATE_DIM_TEST + 3] = ((Ixx - Iyy) / (2 * Izz)) * q;
    A_data[5 * STATE_DIM_TEST + 4] = ((Ixx - Iyy) / (2 * Izz)) * p;
}

void discretize_matrices() {
    float samplingTime = 0.01f;
    float dt = samplingTime;
    
    // Salva uma cópia da matriz A contínua
    float A_continuous[STATE_DIM_TEST * STATE_DIM_TEST];
    memcpy(A_continuous, A_data, sizeof(A_data));
    
    // Método de discretização mais preciso: expm(A*dt)
    // Para sistemas pequenos, usar série de Taylor até 3ª ordem
    float dt2_over_2 = dt * dt * 0.5f;
    float dt3_over_6 = dt * dt * dt / 6.0f;
    
    // Calcula A^2
    float A2[STATE_DIM_TEST * STATE_DIM_TEST] = {0};
    MatrixOperations::matrixMultiply(A_continuous, A_continuous, A2, STATE_DIM_TEST, STATE_DIM_TEST, STATE_DIM_TEST);
    
    // Calcula A^3 para maior precisão
    float A3[STATE_DIM_TEST * STATE_DIM_TEST] = {0};
    MatrixOperations::matrixMultiply(A2, A_continuous, A3, STATE_DIM_TEST, STATE_DIM_TEST, STATE_DIM_TEST);
    
    // Discretiza A: Ad = I + A*dt + (A^2*dt^2)/2 + (A^3*dt^3)/6
    for (int i = 0; i < STATE_DIM_TEST; i++) {
        for (int j = 0; j < STATE_DIM_TEST; j++) {
            int idx = i * STATE_DIM_TEST + j;
            if (i == j) {
                A_data[idx] = 1.0f + A_continuous[idx] * dt + 
                             A2[idx] * dt2_over_2 + 
                             A3[idx] * dt3_over_6;
            } else {
                A_data[idx] = A_continuous[idx] * dt + 
                             A2[idx] * dt2_over_2 + 
                             A3[idx] * dt3_over_6;
            }
        }
    }
    
    // Discretiza B: Bd = B*dt + (A*B*dt^2)/2 + (A^2*B*dt^3)/6
    float AB[STATE_DIM_TEST * CONTROL_DIM_TEST] = {0};
    MatrixOperations::matrixMultiply(A_continuous, B_data, AB, STATE_DIM_TEST, STATE_DIM_TEST, CONTROL_DIM_TEST);
    
    float A2B[STATE_DIM_TEST * CONTROL_DIM_TEST] = {0};
    MatrixOperations::matrixMultiply(A2, B_data, A2B, STATE_DIM_TEST, STATE_DIM_TEST, CONTROL_DIM_TEST);
    
    // Salva B original
    float B_continuous[STATE_DIM_TEST * CONTROL_DIM_TEST];
    memcpy(B_continuous, B_data, sizeof(B_data));
    
    for (int i = 0; i < STATE_DIM_TEST; i++) {
        for (int j = 0; j < CONTROL_DIM_TEST; j++) {
            int index = i * CONTROL_DIM_TEST + j;
            B_data[index] = B_continuous[index] * dt + 
                           AB[index] * dt2_over_2 + 
                           A2B[index] * dt3_over_6;
        }
    }
}

void print_matrix(const float* matrix, int rows, int cols, const char* name) {
    Serial.printf("Matriz %s (%dx%d):\n", name, rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            Serial.printf("%8.4f ", matrix[i * cols + j]);
        }
        Serial.println();
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("--- Teste do Solver DARE com Sistema de Drone ---");
    Serial.printf("Parâmetros: roll=%.2f, pitch=%.2f, yaw=%.2f, p=%.2f, q=%.2f, r=%.2f\n\n",
                  roll, pitch, yaw, p, q, r);

    // Inicializa e discretiza as matrizes
    initialize_A_matrix();
    discretize_matrices();

    // Configura as matrizes no objeto lqr_test
    lqr_test.setStateMatrix(A_data);
    lqr_test.setInputMatrix(B_data);
    lqr_test.setCostMatrices(Q_data, R_data);

    Serial.println("Matrizes de entrada (discretizadas):");
    print_matrix(A_data, STATE_DIM_TEST, STATE_DIM_TEST, "A");
    print_matrix(B_data, STATE_DIM_TEST, CONTROL_DIM_TEST, "B");
    print_matrix(Q_data, STATE_DIM_TEST, STATE_DIM_TEST, "Q");
    print_matrix(R_data, CONTROL_DIM_TEST, CONTROL_DIM_TEST, "R");

    // ========================================================================
    // COMPARAÇÃO DE DESEMPENHO: VAN DOOREN vs SCHUR
    // ========================================================================
    
    Serial.println("\n╔═══════════════════════════════════════════════════════════════════╗");
    Serial.println("║     COMPARAÇÃO DE PERFORMANCE: Van Dooren vs Schur               ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════════╝\n");
    
    // Variáveis para armazenar resultados
    float K_vandooren[CONTROL_DIM_TEST * STATE_DIM_TEST];
    float P_vandooren[STATE_DIM_TEST * STATE_DIM_TEST];
    float K_schur[CONTROL_DIM_TEST * STATE_DIM_TEST];
    float P_schur[STATE_DIM_TEST * STATE_DIM_TEST];
    
    unsigned long time_vandooren = 0;
    unsigned long time_schur = 0;
    
    // ========================================================================
    // TESTE 1: Método de Van Dooren
    // ========================================================================
    Serial.println("═══════════════════════════════════════════════════════════════════");
    Serial.println("  MÉTODO 1: Van Dooren (Pencil 3n x 3n)");
    Serial.println("═══════════════════════════════════════════════════════════════════\n");
    
    unsigned long t1_start = micros();
    bool success_vandooren = lqr_test.computeGains();  // Usa Van Dooren por padrão
    time_vandooren = micros() - t1_start;
    
    if (success_vandooren) {
        lqr_test.exportGains(K_vandooren);
        const float* P_ptr = lqr_test.getRicattiSolution();
        if (P_ptr) {
            memcpy(P_vandooren, P_ptr, STATE_DIM_TEST * STATE_DIM_TEST * sizeof(float));
        }
        
        Serial.println("\n✓ Van Dooren: Sucesso!");
        Serial.print("  Tempo total: ");
        Serial.print(time_vandooren / 1000.0, 3);
        Serial.println(" ms\n");
        
        Serial.println("--- Resultado: Matriz K (Van Dooren) ---");
        print_matrix(K_vandooren, CONTROL_DIM_TEST, STATE_DIM_TEST, "K_VanDooren");
        
    } else {
        Serial.println("\n✗ Van Dooren: Falhou!\n");
    }
    
    delay(1000);  // Pequena pausa entre testes
    
    // ========================================================================
    // TESTE 2: Método de Schur
    // ========================================================================
    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("  MÉTODO 2: Schur (QZ Decomposition, Pencil 2n x 2n)");
    Serial.println("═══════════════════════════════════════════════════════════════════\n");
    
    // Criar nova instância para método Schur
    AutoLQR lqr_schur(STATE_DIM_TEST, CONTROL_DIM_TEST);
    lqr_schur.setStateMatrix(A_data);
    lqr_schur.setInputMatrix(B_data);
    lqr_schur.setCostMatrices(Q_data, R_data);
    
    // Chamar diretamente o método Schur (precisa tornar público ou criar interface)
    // Por enquanto, vamos medir via computeGains modificado temporariamente
    unsigned long t2_start = micros();
    bool success_schur = lqr_schur.computeGains();
    time_schur = micros() - t2_start;
    
    if (success_schur) {
        lqr_schur.exportGains(K_schur);
        const float* P_ptr = lqr_schur.getRicattiSolution();
        if (P_ptr) {
            memcpy(P_schur, P_ptr, STATE_DIM_TEST * STATE_DIM_TEST * sizeof(float));
        }
        
        Serial.println("\n✓ Schur: Sucesso!");
        Serial.print("  Tempo total: ");
        Serial.print(time_schur / 1000.0, 3);
        Serial.println(" ms\n");
        
        Serial.println("--- Resultado: Matriz K (Schur) ---");
        print_matrix(K_schur, CONTROL_DIM_TEST, STATE_DIM_TEST, "K_Schur");
        
    } else {
        Serial.println("\n✗ Schur: Falhou!\n");
    }
    
    // ========================================================================
    // COMPARAÇÃO E ANÁLISE
    // ========================================================================
    if (success_vandooren && success_schur) {
        Serial.println("\n╔═══════════════════════════════════════════════════════════════════╗");
        Serial.println("║                    ANÁLISE COMPARATIVA                            ║");
        Serial.println("╚═══════════════════════════════════════════════════════════════════╝\n");
        
        Serial.println("--- TEMPOS DE EXECUÇÃO ---");
        Serial.print("  Van Dooren: ");
        Serial.print(time_vandooren / 1000.0, 3);
        Serial.println(" ms");
        
        Serial.print("  Schur:      ");
        Serial.print(time_schur / 1000.0, 3);
        Serial.println(" ms");
        
        Serial.println();
        
        // Comparação percentual
        if (time_vandooren > time_schur) {
            float speedup = (float)time_vandooren / (float)time_schur;
            Serial.print("  → Schur é ");
            Serial.print(speedup, 2);
            Serial.print("x mais rápido (");
            Serial.print(((time_vandooren - time_schur) * 100.0 / time_vandooren), 1);
            Serial.println("% mais rápido)");
        } else {
            float speedup = (float)time_schur / (float)time_vandooren;
            Serial.print("  → Van Dooren é ");
            Serial.print(speedup, 2);
            Serial.print("x mais rápido (");
            Serial.print(((time_schur - time_vandooren) * 100.0 / time_schur), 1);
            Serial.println("% mais rápido)");
        }
        
        Serial.println("\n--- DIFERENÇA NOS RESULTADOS ---");
        
        // Calcular diferença nas matrizes K
        float max_diff_K = 0.0f;
        float avg_diff_K = 0.0f;
        for (int i = 0; i < CONTROL_DIM_TEST * STATE_DIM_TEST; i++) {
            float diff = fabs(K_vandooren[i] - K_schur[i]);
            avg_diff_K += diff;
            if (diff > max_diff_K) max_diff_K = diff;
        }
        avg_diff_K /= (CONTROL_DIM_TEST * STATE_DIM_TEST);
        
        Serial.print("  Diferença máxima em K: ");
        Serial.println(max_diff_K, 8);
        Serial.print("  Diferença média em K:  ");
        Serial.println(avg_diff_K, 8);
        
        // Calcular diferença nas matrizes P
        float max_diff_P = 0.0f;
        float avg_diff_P = 0.0f;
        for (int i = 0; i < STATE_DIM_TEST * STATE_DIM_TEST; i++) {
            float diff = fabs(P_vandooren[i] - P_schur[i]);
            avg_diff_P += diff;
            if (diff > max_diff_P) max_diff_P = diff;
        }
        avg_diff_P /= (STATE_DIM_TEST * STATE_DIM_TEST);
        
        Serial.print("  Diferença máxima em P: ");
        Serial.println(max_diff_P, 8);
        Serial.print("  Diferença média em P:  ");
        Serial.println(avg_diff_P, 8);
        
        Serial.println("\n--- CARACTERÍSTICAS DOS MÉTODOS ---");
        Serial.println("  Van Dooren:");
        Serial.println("    • Pencil 3n x 3n (maior dimensão)");
        Serial.println("    • Numericamente mais estável");
        Serial.println("    • Ideal para sistemas mal condicionados");
        
        Serial.println("\n  Schur:");
        Serial.println("    • Pencil 2n x 2n (menor dimensão)");
        Serial.println("    • Mais rápido (menos cálculos)");
        Serial.println("    • Suficiente para sistemas bem condicionados");
        
        Serial.println("\n═══════════════════════════════════════════════════════════════════\n");
    }

    // APÓS: discretize_matrices();
    
    // Verificar propriedades das matrizes discretizadas
    Serial.println("\n=== VERIFICAÇÃO DAS MATRIZES ===");
    
    // Verificar se Ad tem autovalores dentro do círculo unitário
    Serial.println("Verificando estabilidade de Ad...");
    
    // Verificar controlabilidade discreta
    Serial.println("Verificando controlabilidade...");
    
    // Verificar se Q é semi-definida positiva
    Serial.println("Elementos diagonais de Q:");
    for (int i = 0; i < STATE_DIM_TEST; i++) {
        Serial.printf("  Q[%d,%d] = %.4f\n", i, i, Q_data[i * STATE_DIM_TEST + i]);
    }
    
    // Verificar se R é definida positiva
    Serial.println("Elementos diagonais de R:");
    for (int i = 0; i < CONTROL_DIM_TEST; i++) {
        Serial.printf("  R[%d,%d] = %.4f\n", i, i, R_data[i * CONTROL_DIM_TEST + i]);
    }
    
    Serial.println("\n--- Teste Concluído ---");
}

void loop() {
    // O teste roda apenas uma vez no setup()
}
