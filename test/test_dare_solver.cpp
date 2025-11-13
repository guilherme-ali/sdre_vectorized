#include <Arduino.h>
#include "AutoLQR.h"
#include "MatrixOperations.h"

// Define as dimensões das matrizes para o exemplo
// Sistema: x(k+1) = Ax(k) + Bu(k)
// A: 6x6, B: 6x3, Q: 6x6, R: 3x3
const int STATE_DIM_TEST = 6;
const int CONTROL_DIM_TEST = 3;

// Instância do controlador LQR para o teste
AutoLQR lqr_test(STATE_DIM_TEST, CONTROL_DIM_TEST);

// Matrizes do problema DARE de exemplo
// Sistema 6x6 com 3 entradas de controle
float A_data[STATE_DIM_TEST * STATE_DIM_TEST] = {
    0.9,  0.1,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.8,  0.2,  0.0,  0.0,  0.0,
    0.1,  0.0,  0.85, 0.1,  0.0,  0.0,
    0.0,  0.1,  0.0,  0.9,  0.1,  0.0,
    0.0,  0.0,  0.1,  0.0,  0.88, 0.1,
    0.0,  0.0,  0.0,  0.1,  0.0,  0.92
};

float B_data[STATE_DIM_TEST * CONTROL_DIM_TEST] = {
    1.0,  0.0,  0.0,
    0.5,  0.5,  0.0,
    0.0,  1.0,  0.3,
    0.0,  0.3,  0.7,
    0.2,  0.0,  1.0,
    0.0,  0.0,  0.5
};

float Q_data[STATE_DIM_TEST * STATE_DIM_TEST] = {
    10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
     0.0,  8.0,  0.0,  0.0,  0.0,  0.0,
     0.0,  0.0,  5.0,  0.0,  0.0,  0.0,
     0.0,  0.0,  0.0,  3.0,  0.0,  0.0,
     0.0,  0.0,  0.0,  0.0,  2.0,  0.0,
     0.0,  0.0,  0.0,  0.0,  0.0,  1.0
};

float R_data[CONTROL_DIM_TEST * CONTROL_DIM_TEST] = {
    1.0,  0.0,  0.0,
    0.0,  1.0,  0.0,
    0.0,  0.0,  1.0
};

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
    delay(2000); // Aguarda a porta serial estabilizar

    Serial.println("--- Teste do Solver DARE (Método Schur/QZ) ---");

    // Configura as matrizes no objeto lqr_test
    lqr_test.setStateMatrix(A_data);
    lqr_test.setInputMatrix(B_data);
    lqr_test.setCostMatrices(Q_data, R_data);

    Serial.println("Matrizes de entrada:");
    print_matrix(A_data, STATE_DIM_TEST, STATE_DIM_TEST, "A");
    print_matrix(B_data, STATE_DIM_TEST, CONTROL_DIM_TEST, "B");
    print_matrix(Q_data, STATE_DIM_TEST, STATE_DIM_TEST, "Q");
    print_matrix(R_data, CONTROL_DIM_TEST, CONTROL_DIM_TEST, "R");

    // Calcula os ganhos usando o novo método
    Serial.println("Calculando a solução da DARE e a matriz de ganho K...");
    bool success = lqr_test.computeGains();

    if (success) {
        Serial.println("\n--- Resultados ---");
        
        // Obtém e imprime a matriz de ganho K
        float K_result[CONTROL_DIM_TEST * STATE_DIM_TEST];
        lqr_test.exportGains(K_result);
        print_matrix(K_result, CONTROL_DIM_TEST, STATE_DIM_TEST, "K (Ganho)");

        // Obtém e imprime a matriz P (solução da DARE)
        const float* P_ptr = lqr_test.getRicattiSolution();
        if (P_ptr) {
            print_matrix(P_ptr, STATE_DIM_TEST, STATE_DIM_TEST, "P (Solução de Riccati)");
        }

    } else {
        Serial.println("Falha ao calcular a solução da DARE. Verifique as mensagens de erro.");
    }

    Serial.println("\n--- Teste Concluído ---");
}

void loop() {
    // O teste roda apenas uma vez no setup()
}
