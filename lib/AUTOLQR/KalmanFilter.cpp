#include "KalmanFilter.h"
#include <new> // Para std::nothrow

KalmanFilter::KalmanFilter(int state_dim, int control_dim, int measurement_dim)
    : n(state_dim), p(control_dim), m(measurement_dim) {

    // Alocar memória para todas as matrizes e vetores
    A = new(std::nothrow) float[n * n];
    B = new(std::nothrow) float[n * p];
    C = new(std::nothrow) float[m * n];
    Q = new(std::nothrow) float[n * n];
    R = new(std::nothrow) float[m * m];
    A_T = new(std::nothrow) float[n * n];
    C_T = new(std::nothrow) float[n * m];
    
    x_est = new(std::nothrow) float[n];
    P_est = new(std::nothrow) float[n * n];
    
    x_pred = new(std::nothrow) float[n];
    P_pred = new(std::nothrow) float[n * n];
    
    K = new(std::nothrow) float[n * m];
    y = new(std::nothrow) float[m];
    
    // Alocar buffers temporários
    temp_n1_1 = new(std::nothrow) float[n];
    temp_n1_2 = new(std::nothrow) float[n];
    temp_m1_1 = new(std::nothrow) float[m];
    temp_nn_1 = new(std::nothrow) float[n * n];
    temp_nn_2 = new(std::nothrow) float[n * n];
    temp_mn_1 = new(std::nothrow) float[m * n];
    temp_nm_1 = new(std::nothrow) float[n * m];
    temp_mm_1 = new(std::nothrow) float[m * m];
    temp_mm_2 = new(std::nothrow) float[m * m];
    S_inv = new(std::nothrow) float[m * m];
}

KalmanFilter::~KalmanFilter() {
    // Liberar toda a memória alocada
    delete[] A; delete[] B; delete[] C;
    delete[] Q; delete[] R; delete[] A_T; delete[] C_T;
    delete[] x_est; delete[] P_est;
    delete[] x_pred; delete[] P_pred;
    delete[] K; delete[] y;
    
    delete[] temp_n1_1; delete[] temp_n1_2; delete[] temp_m1_1;
    delete[] temp_nn_1; delete[] temp_nn_2;
    delete[] temp_mn_1; delete[] temp_nm_1;
    delete[] temp_mm_1; delete[] temp_mm_2; delete[] S_inv;
}

void KalmanFilter::init(const float* A_init, const float* B_init, const float* C_init,
                        const float* Q_init, const float* R_init, const float* x0_init, const float* P0_init) {
    // Copiar matrizes do modelo
    MatrixOperations::matrixCopy(A_init, A, n * n);
    MatrixOperations::matrixCopy(B_init, B, n * p);
    MatrixOperations::matrixCopy(C_init, C, m * n);
    MatrixOperations::matrixCopy(Q_init, Q, n * n);
    MatrixOperations::matrixCopy(R_init, R, m * m);

    // Pré-calcular transpostas
    MatrixOperations::transposeMatrix(A, A_T, n, n);
    MatrixOperations::transposeMatrix(C, C_T, m, n);

    // Inicializar estado e covariância
    MatrixOperations::matrixCopy(x0_init, x_est, n);
    MatrixOperations::matrixCopy(P0_init, P_est, n * n);
}

void KalmanFilter::predict(const float* u) {
    // 1. Predição do Estado: x_pred = A * x_est + B * u
    MatrixOperations::matrixVectorMultiply(A, x_est, temp_n1_1, n, n); // A * x_est
    if (u != nullptr && p > 0) {
        MatrixOperations::matrixVectorMultiply(B, u, temp_n1_2, n, p); // B * u
        MatrixOperations::matrixAdd(temp_n1_1, temp_n1_2, x_pred, n, 1); // (A*x_est) + (B*u)
    } else {
        MatrixOperations::matrixCopy(temp_n1_1, x_pred, n);
    }
    
    // 2. Predição da Covariância: P_pred = A * P_est * A' + Q
    MatrixOperations::matrixMultiply(A, P_est, temp_nn_1, n, n, n);       // A * P_est
    MatrixOperations::matrixMultiply(temp_nn_1, A_T, temp_nn_2, n, n, n); // (A * P_est) * A'
    MatrixOperations::matrixAdd(temp_nn_2, Q, P_pred, n, n);              // (A * P_est * A') + Q
}

void KalmanFilter::update(const float* z) {
    // 1. Cálculo do Ganho de Kalman: K = P_pred * C' * inv(C * P_pred * C' + R)
    //    Cálculo de S = C * P_pred * C' + R
    MatrixOperations::matrixMultiply(C, P_pred, temp_mn_1, m, n, n);       // C * P_pred
    MatrixOperations::matrixMultiply(temp_mn_1, C_T, temp_mm_1, m, n, m);  // (C * P_pred) * C'
    MatrixOperations::matrixAdd(temp_mm_1, R, temp_mm_2, m, m);            // S = (C * P_pred * C') + R

    //    Inversão de S
    if (!MatrixOperations::invertMatrix(temp_mm_2, S_inv, m)) {
        // A matriz é singular, não é possível atualizar.
        // Copia predição para estimativa para não corromper o estado
        MatrixOperations::matrixCopy(x_pred, x_est, n);
        MatrixOperations::matrixCopy(P_pred, P_est, n * n);
        return;
    }

    //    Cálculo final de K
    MatrixOperations::matrixMultiply(P_pred, C_T, temp_nm_1, n, n, m);     // P_pred * C'
    MatrixOperations::matrixMultiply(temp_nm_1, S_inv, K, n, m, m);        // K = (P_pred * C') * inv(S)

    // 2. Atualização do Estado: x_est = x_pred + K * (z - C * x_pred)
    //    Cálculo da inovação y = z - C * x_pred
    MatrixOperations::matrixVectorMultiply(C, x_pred, temp_m1_1, m, n);   // C * x_pred
    MatrixOperations::matrixSubtract(z, temp_m1_1, y, m, 1);              // y = z - (C * x_pred)
    
    //    Cálculo de x_est
    MatrixOperations::matrixVectorMultiply(K, y, temp_n1_1, n, m);        // K * y
    MatrixOperations::matrixAdd(x_pred, temp_n1_1, x_est, n, 1);          // x_est = x_pred + (K * y)

    // 3. Atualização da Covariância: P_est = P_pred - K * C * P_pred
    //    (Usando a forma padrão, não a de Joseph, por simplicidade)
    MatrixOperations::matrixMultiply(K, C, temp_nn_1, n, m, n);           // K * C
    MatrixOperations::matrixMultiply(temp_nn_1, P_pred, temp_nn_2, n, n, n); // (K * C) * P_pred
    MatrixOperations::matrixSubtract(P_pred, temp_nn_2, P_est, n, n);     // P_est = P_pred - (K * C * P_pred)
}