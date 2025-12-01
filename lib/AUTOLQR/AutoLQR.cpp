#include "AutoLQR.h"
#include <math.h>
#include <ArduinoEigen.h>
#include <string.h>  // Para strcmp

AutoLQR::AutoLQR(int stateSize, int controlSize)
    : stateSize(stateSize)
    , controlSize(controlSize)
    , A(nullptr)
    , B(nullptr)
    , Q(nullptr)
    , R(nullptr)
    , K(nullptr)
    , state(nullptr)
    , P(nullptr)
    , Kr(nullptr)
    , reference(nullptr)
{
    if (stateSize > 0 && controlSize > 0) {
        A = new float[stateSize * stateSize]();
        B = new float[stateSize * controlSize]();
        Q = new float[stateSize * stateSize]();
        R = new float[controlSize * controlSize]();
        K = new float[controlSize * stateSize]();
        state = new float[stateSize]();
        P = new float[stateSize * stateSize]();
        Kr = new float[controlSize * controlSize]();
        reference = new float[controlSize]();
    }
}

AutoLQR::~AutoLQR()
{
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] K;
    delete[] state;
    delete[] P;
    delete[] Kr;
    delete[] reference;
}

bool AutoLQR::setStateMatrix(const float* inputA)
{
    if (!inputA || !A)
        return false;
    matrixCopy(inputA, A, stateSize * stateSize);
    return true;
}

bool AutoLQR::setInputMatrix(const float* inputB)
{
    if (!inputB || !B)
        return false;
    matrixCopy(inputB, B, stateSize * controlSize);
    return true;
}

bool AutoLQR::setCostMatrices(const float* inputQ, const float* inputR)
{
    if (!inputQ || !inputR || !Q || !R)
        return false;
    matrixCopy(inputQ, Q, stateSize * stateSize);
    matrixCopy(inputR, R, controlSize * controlSize);
    return true;
}

void AutoLQR::setGains(const float* inputK)
{
    if (!inputK || !K)
        return;
    matrixCopy(inputK, K, controlSize * stateSize);
}

bool AutoLQR::computeGains(const char* method)
{
    bool K_flag = false;
    
    if (strcmp(method, "SDA") == 0) {
        K_flag = computeGainMatrixSDA();
    } else if (strcmp(method, "SCHUR") == 0) {
        K_flag = computeGainMatrixSchur();
    } else if (strcmp(method, "VAN_DOOREN") == 0) {
        K_flag = computeGainMatrixVanDooren();
    } else if (strcmp(method, "ITERATIVE") == 0) {
        K_flag = computeGainMatrixIterative();
    } else {
        // Método padrão: SDA
        Serial.print(F("Método desconhecido: "));
        Serial.print(method);
        Serial.println(F(". Usando SDA."));
        K_flag = computeGainMatrixSDA();
    }
    
    if (!K_flag)
        return false;

    bool Kr_flag = computeGainMatrixKr();
    return Kr_flag;
}

void AutoLQR::updateState(const float* currentState)
{
    if (!currentState || !state)
        return;
    matrixCopy(currentState, state, stateSize);
}

void AutoLQR::updateReference(const float* newReference)
{
    if (!newReference || !reference)
        return;
    matrixCopy(newReference, reference, controlSize);
}

void AutoLQR::calculateControl(float* controlOutput)
{
    if (!controlOutput || !K || !state || !Kr || !reference)
        return;

    // Initialize control outputs to zero
    matrixClear(controlOutput, controlSize);

    // u = -K·x + Kr·r
    for (int i = 0; i < controlSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            controlOutput[i] -= K[i * stateSize + j] * state[j];
            if(j < controlSize) {
                controlOutput[i] += Kr[i * controlSize + j] * reference[j];
            }
        }
    }
}

bool AutoLQR::isSystemControllable()
{
    // Basic controllability check for 2x2 systems
    if (stateSize == 2 && controlSize == 1) {
        float det = B[0] * A[1] - B[1] * A[0];
        return fabs(det) > 1e-6;
    }

    // For larger systems, implement a more sophisticated controllability check
    // or return true and let the DARE solver determine feasibility
    return true;
}

const float* AutoLQR::getRicattiSolution() const
{
    return P;
}

bool AutoLQR::computeGainMatrixSDA()
{
    // Implementação do Structure-preserving Doubling Algorithm (SDA)
    // Para DARE: A'·P·A - P - A'·P·B·(R + B'·P·B)^(-1)·B'·P·A + Q = 0
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    // Alocação de memória
    float* Ak = new float[stateSize * stateSize]();
    float* Gk = new float[stateSize * stateSize]();
    float* Hk = new float[stateSize * stateSize]();
    
    float* Ak_next = new float[stateSize * stateSize]();
    float* Gk_next = new float[stateSize * stateSize]();
    float* Hk_next = new float[stateSize * stateSize]();
    
    float* R_inv = new float[controlSize * controlSize]();
    float* BT = new float[controlSize * stateSize]();
    float* AT = new float[stateSize * stateSize]();
    float* W = new float[stateSize * stateSize]();
    float* Temp1 = new float[stateSize * stateSize]();
    float* Temp2 = new float[stateSize * stateSize]();
    float* Temp3 = new float[stateSize * stateSize]();
    
    // ========================================================================
    // INICIALIZAÇÃO CORRETA DO SDA PARA DARE
    // ========================================================================
    
    // 1. Ak = A (correto)
    matrixCopy(A, Ak, stateSize * stateSize);
    
    // 2. Calcular transpostas
    transposeMatrix(A, AT, stateSize, stateSize);
    transposeMatrix(B, BT, stateSize, controlSize);
    
    // 3. Calcular R_inv
    matrixCopy(R, R_inv, controlSize * controlSize);
    if (!invertMatrix(R_inv, R_inv, controlSize)) {
        delete[] Ak; delete[] Gk; delete[] Hk;
        delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
        return false;
    }
    
    // 4. Gk = B * R^(-1) * B' (correto)
    float* B_Rinv = new float[stateSize * controlSize];
    matrixMultiply(B, R_inv, B_Rinv, stateSize, controlSize, controlSize);
    matrixMultiply(B_Rinv, BT, Gk, stateSize, controlSize, stateSize);
    delete[] B_Rinv;

    // 5. CORREÇÃO CRÍTICA: Hk = Q (não A'·Q·A + Q)
    matrixCopy(Q, Hk, stateSize * stateSize);

    // ========================================================================
    // LOOP SDA
    // ========================================================================
    const int maxIterations = 5000;
    const float tolerance = 1e-9f;
    bool converged = false;

    for (int iter = 0; iter < maxIterations; iter++) {
        // W = (I + Gk·Hk)^(-1)
        matrixMultiply(Gk, Hk, Temp1, stateSize, stateSize, stateSize);
        
        for (int i = 0; i < stateSize; i++) {
            Temp1[i * stateSize + i] += 1.0f;
        }
        
        matrixCopy(Temp1, W, stateSize * stateSize);
        if (!invertMatrix(W, W, stateSize)) {
            break;
        }
        
        // Temp1 = Ak·W
        matrixMultiply(Ak, W, Temp1, stateSize, stateSize, stateSize);
        
        // Ak_next = Temp1·Ak = (Ak·W)·Ak
        matrixMultiply(Temp1, Ak, Ak_next, stateSize, stateSize, stateSize);
        
        // Gk_next = Gk + (Ak·W)·Gk·Ak'
        transposeMatrix(Ak, AT, stateSize, stateSize);
        matrixMultiply(Gk, AT, Temp2, stateSize, stateSize, stateSize);
        matrixMultiply(Temp1, Temp2, Temp3, stateSize, stateSize, stateSize);
        matrixAdd(Gk, Temp3, Gk_next, stateSize, stateSize);
        
        // Hk_next = Hk + Ak'·Hk·W·Ak
        matrixMultiply(W, Ak, Temp2, stateSize, stateSize, stateSize);
        matrixMultiply(Hk, Temp2, Temp3, stateSize, stateSize, stateSize);
        matrixMultiply(AT, Temp3, Temp2, stateSize, stateSize, stateSize);
        matrixAdd(Hk, Temp2, Hk_next, stateSize, stateSize);
        
        // Verificar convergência
        float diff = 0;
        for (int i = 0; i < stateSize * stateSize; i++) {
            diff += fabsf(Hk_next[i] - Hk[i]);
        }
        
        // Atualizar
        matrixCopy(Ak_next, Ak, stateSize * stateSize);
        matrixCopy(Gk_next, Gk, stateSize * stateSize);
        matrixCopy(Hk_next, Hk, stateSize * stateSize);
        
        if (diff < tolerance) {
            converged = true;
            break;
        }
    }

    // P = Hk (solução final)
    matrixCopy(Hk, P, stateSize * stateSize);

    // ========================================================================
    // CÁLCULO DO GANHO K
    // ========================================================================
    // K = (R + B'·P·B)^(-1) · B'·P·A
    
    float* BT_P = new float[controlSize * stateSize];
    float* BT_P_B = new float[controlSize * controlSize];
    float* BT_P_A = new float[controlSize * stateSize];
    float* R_plus_BTPB = new float[controlSize * controlSize];
    
    // BT_P = B'·P
    matrixMultiply(BT, P, BT_P, controlSize, stateSize, stateSize);
    
    // BT_P_B = (B'·P)·B
    matrixMultiply(BT_P, B, BT_P_B, controlSize, stateSize, controlSize);
    
    // R_plus_BTPB = R + B'·P·B
    matrixAdd(R, BT_P_B, R_plus_BTPB, controlSize, controlSize);
    
    // Inverter
    if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, controlSize)) {
        converged = false;
    } else {
        // BT_P_A = (B'·P)·A
        matrixMultiply(BT_P, A, BT_P_A, controlSize, stateSize, stateSize);
        
        // K = (R + B'·P·B)^(-1) · (B'·P·A)
        matrixMultiply(R_plus_BTPB, BT_P_A, K, controlSize, controlSize, stateSize);
    }

    // Limpeza
    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
    delete[] BT_P; delete[] BT_P_B; delete[] BT_P_A; delete[] R_plus_BTPB;

    return converged;
}

bool AutoLQR::computeGainMatrixSchur()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int n2 = 2 * n;
    
    // ========================================================================
    // PASSO 1: Alocar memória para matrizes intermediárias
    // ========================================================================
    float* R_inv = new float[controlSize * controlSize];
    float* BT = new float[controlSize * stateSize];
    float* AT = new float[stateSize * stateSize];
    float* temp_ctrl_state = new float[controlSize * stateSize];
    float* temp_state_ctrl = new float[stateSize * controlSize];
    float* G = new float[stateSize * stateSize];
    
    // ========================================================================
    // PASSO 2: Calcular G = B * inv(R) * B^T usando operações manuais
    // ========================================================================
    matrixCopy(R, R_inv, controlSize * controlSize);
    if (!invertMatrix(R_inv, R_inv, controlSize)) {
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        return false;
    }
    
    matrixMultiply(B, R_inv, temp_state_ctrl, stateSize, controlSize, controlSize);
    transposeMatrix(B, BT, stateSize, controlSize);
    matrixMultiply(temp_state_ctrl, BT, G, stateSize, controlSize, stateSize);
    
    // ========================================================================
    // PASSO 3: Calcular A^T
    // ========================================================================
    transposeMatrix(A, AT, stateSize, stateSize);
    
    // ========================================================================
    // PASSO 4: Construir matrizes H e J (2n x 2n) - USANDO EIGEN
    // ========================================================================
    Eigen::MatrixXf H(n2, n2);
    Eigen::MatrixXf J(n2, n2);
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(i, j) = A[i * n + j];
            H(i, j + n) = 0.0f;
            H(i + n, j) = -Q[i * n + j];
            H(i + n, j + n) = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            J(i, j) = (i == j) ? 1.0f : 0.0f;
            J(i, j + n) = G[i * n + j];
            J(i + n, j) = 0.0f;
            J(i + n, j + n) = AT[i * n + j];
        }
    }
    
    // ========================================================================
    // PASSO 5: Decomposição QZ
    // ========================================================================
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> ges;
    ges.compute(H, J, true);
    
    if (ges.info() != Eigen::Success) {
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        return false;
    }
    
    Eigen::VectorXcf alpha = ges.alphas();
    Eigen::VectorXcf beta = ges.betas();
    
    // ========================================================================
    // PASSO 6: Ordenar autovetores estáveis (|λ| < 1)
    // ========================================================================
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    for (int i = 0; i < n2; i++) {
        if (std::abs(beta(i)) > 1e-10f) {
            std::complex<float> eigenvalue = alpha(i) / beta(i);
            float magnitude = std::abs(eigenvalue);
            
            if (magnitude < 1.0f) {
                stable_indices.push_back(i);
            }
        }
    }
    
    if (stable_indices.size() != static_cast<size_t>(n)) {
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        return false;
    }
    
    // ========================================================================
    // PASSO 7: Extrair subespaço invariante estável
    // ========================================================================
    Eigen::MatrixXcf Z = ges.eigenvectors();
    
    float* U11_real = new float[n * n];
    float* U11_imag = new float[n * n];
    float* U21_real = new float[n * n];
    float* U21_imag = new float[n * n];
    
    for (int j = 0; j < n; j++) {
        int idx = stable_indices[j];
        for (int i = 0; i < n; i++) {
            U11_real[i * n + j] = Z(i, idx).real();
            U11_imag[i * n + j] = Z(i, idx).imag();
            U21_real[i * n + j] = Z(i + n, idx).real();
            U21_imag[i * n + j] = Z(i + n, idx).imag();
        }
    }
    
    // ========================================================================
    // PASSO 8: Calcular P = U21 * inv(U11)
    // ========================================================================
    Eigen::MatrixXcf U11_complex(n, n);
    Eigen::MatrixXcf U21_complex(n, n);
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            U11_complex(i, j) = std::complex<float>(U11_real[i * n + j], U11_imag[i * n + j]);
            U21_complex(i, j) = std::complex<float>(U21_real[i * n + j], U21_imag[i * n + j]);
        }
    }
    
    Eigen::MatrixXcf P_complex = U21_complex * U11_complex.inverse();
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i * n + j] = P_complex(i, j).real();
        }
    }
    
    // Forçar simetria de P
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
            P[i * n + j] = avg;
            P[j * n + i] = avg;
        }
    }
    
    // ========================================================================
    // PASSO 9: Calcular K = (R + B^T*P*B)^{-1} * B^T*P*A
    // ========================================================================
    float* BT_P = new float[controlSize * stateSize];
    matrixMultiply(BT, P, BT_P, controlSize, stateSize, stateSize);
    
    float* BT_P_B = new float[controlSize * controlSize];
    matrixMultiply(BT_P, B, BT_P_B, controlSize, stateSize, controlSize);
    
    float* term = new float[controlSize * controlSize];
    matrixAdd(R, BT_P_B, term, controlSize, controlSize);
    
    if (!invertMatrix(term, term, controlSize)) {
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        delete[] U11_real; delete[] U11_imag; delete[] U21_real; delete[] U21_imag;
        delete[] BT_P; delete[] BT_P_B; delete[] term;
        return false;
    }
    
    float* BT_P_A = new float[controlSize * stateSize];
    matrixMultiply(BT_P, A, BT_P_A, controlSize, stateSize, stateSize);
    
    matrixMultiply(term, BT_P_A, K, controlSize, controlSize, stateSize);

    // ========================================================================
    // Limpeza de memória
    // ========================================================================
    delete[] R_inv;
    delete[] BT;
    delete[] AT;
    delete[] temp_ctrl_state;
    delete[] temp_state_ctrl;
    delete[] G;
    delete[] U11_real;
    delete[] U11_imag;
    delete[] U21_real;
    delete[] U21_imag;
    delete[] BT_P;
    delete[] BT_P_B;
    delete[] term;
    delete[] BT_P_A;
    
    return true;
}

bool AutoLQR::computeGainMatrixVanDooren()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int pencil_size = 2*n + m;

    // ========================================================================
    // Pré-alocar toda memória de uma vez
    // ========================================================================
    const int total_floats = (n*n) + (m*n) + (m*n) + (m*m) + (m*m) + (m*n);
    float* memory_pool = new float[total_floats];
    
    float* AT = memory_pool;
    float* BT = AT + (n*n);
    float* BT_P = BT + (m*n);
    float* BT_P_B = BT_P + (m*n);
    float* term = BT_P_B + (m*m);
    float* BT_P_A = term + (m*m);

    // ========================================================================
    // Calcular transpostas
    // ========================================================================
    if (n == 2) {
        AT[0] = A[0]; AT[1] = A[2];
        AT[2] = A[1]; AT[3] = A[3];
    } else {
        transposeMatrix(A, AT, n, n);
    }
    
    if (n == 2 && m == 1) {
        BT[0] = B[0];
        BT[1] = B[1];
    } else {
        transposeMatrix(B, BT, n, m);
    }

    // ========================================================================
    // Construir pencil
    // ========================================================================
    Eigen::MatrixXf H(pencil_size, pencil_size);
    Eigen::MatrixXf J(pencil_size, pencil_size);
    
    H.setZero();
    J.setZero();

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(i, j) = A[i * n + j];
        }
    }
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            H(i, 2*n + j) = B[i * m + j];
        }
    }
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(n + i, j) = -Q[i * n + j];
        }
    }
    
    for (int i = 0; i < n; i++) {
        H(n + i, n + i) = 1.0f;
    }
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            H(2*n + i, 2*n + j) = R[i * m + j];
        }
    }
    
    for (int i = 0; i < n; i++) {
        J(i, i) = 1.0f;
    }
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            J(n + i, n + j) = AT[i * n + j];
        }
    }
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            J(2*n + i, n + j) = -BT[i * n + j];
        }
    }

    // ========================================================================
    // Decomposição QZ
    // ========================================================================
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> ges;
    ges.compute(H, J, true);
    
    if (ges.info() != Eigen::Success) {
        delete[] memory_pool;
        return false;
    }
    
    Eigen::VectorXcf alpha = ges.alphas();
    Eigen::VectorXcf beta = ges.betas();

    // ========================================================================
    // Seleção de autovalores estáveis
    // ========================================================================
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    const float stability_threshold = 1.0f;
    const float beta_min = 1e-10f;
    
    for (int i = 0; i < pencil_size && stable_indices.size() < static_cast<size_t>(n); i++) {
        const float beta_abs = std::abs(beta(i));
        
        if (beta_abs > beta_min) {
            const std::complex<float> eigenvalue = alpha(i) / beta(i);
            const float magnitude = std::abs(eigenvalue);
            
            if (magnitude < stability_threshold) {
                stable_indices.push_back(i);
            }
        }
    }
    
    if (stable_indices.size() != static_cast<size_t>(n)) {
        delete[] memory_pool;
        return false;
    }

    // ========================================================================
    // Extração do subespaço
    // ========================================================================
    const Eigen::MatrixXcf& Z = ges.eigenvectors();
    
    Eigen::MatrixXcf U1(n, n);
    Eigen::MatrixXcf U2(n, n);
    
    for (int j = 0; j < n; j++) {
        const int idx = stable_indices[j];
        U1.col(j) = Z.col(idx).head(n);
        U2.col(j) = Z.col(idx).segment(n, n);
    }

    // ========================================================================
    // Cálculo de P
    // ========================================================================
    Eigen::MatrixXcf P_complex = U2 * U1.inverse();
    
    for (int i = 0; i < n; i++) {
        const int row_offset = i * n;
        for (int j = 0; j < n; j++) {
            P[row_offset + j] = P_complex(i, j).real();
        }
    }
    
    // Forçar simetria
    for (int i = 0; i < n; i++) {
        const int row_i = i * n;
        for (int j = i + 1; j < n; j++) {
            const float avg = (P[row_i + j] + P[j * n + i]) * 0.5f;
            P[row_i + j] = avg;
            P[j * n + i] = avg;
        }
    }

    // ========================================================================
    // Cálculo de K
    // ========================================================================
    matrixMultiply(BT, P, BT_P, m, n, n);
    matrixMultiply(BT_P, B, BT_P_B, m, n, m);
    matrixAdd(R, BT_P_B, term, m, m);
    
    if (!invertMatrix(term, term, m)) {
        delete[] memory_pool;
        return false;
    }
    
    matrixMultiply(BT_P, A, BT_P_A, m, n, n);
    matrixMultiply(term, BT_P_A, K, m, m, n);

    // ========================================================================
    // Limpeza
    // ========================================================================
    delete[] memory_pool;
    
    return true;
}

bool AutoLQR::computeGainMatrixKr()
{
    // Primeiro, certifique-se de que os ganhos K foram calculados
    if (!K) {
        return false;
    }

    for (int i = 0; i < controlSize; ++i) { 
        for (int j = 0; j < controlSize; ++j) { 
            if (j < stateSize) {
                Kr[i * controlSize + j] = -K[i * stateSize + j];
            } else {
                Kr[i * controlSize + j] = 0.0f;
            }
        }
    }
    return true; 
}

bool AutoLQR::exportKr(float* exportedKr)
{
    if (!Kr || !exportedKr)
        return false;
    
    matrixCopy(Kr, exportedKr, controlSize * controlSize);
    return true;
}

void AutoLQR::estimateFeedforwardGain(float* ffGain, const float* desiredState)
{
    if (!ffGain || !desiredState || !A || !B || !K)
        return;

    // For steady-state tracking: u_ff = -(inv(B'B) * B' * (A-I)) * x_desired
    // This is simplified for common cases

    if (stateSize == 2 && controlSize == 1) {
        // Special case for position-velocity systems
        float Bsq = B[0] * B[0] + B[1] * B[1];
        if (Bsq < 1e-6)
            return;

        float invBsq = 1.0f / Bsq;

        // Compute (A-I) * x_desired
        float dx[2];
        dx[0] = (A[0] - 1.0f) * desiredState[0] + A[1] * desiredState[1];
        dx[1] = A[2] * desiredState[0] + (A[3] - 1.0f) * desiredState[1];

        // Compute feedforward gain
        ffGain[0] = -invBsq * (B[0] * dx[0] + B[1] * dx[1]);
    } else {
        // For other systems, initialize to zero
        matrixClear(ffGain, controlSize);
    }
}

float AutoLQR::estimateConvergenceTime(float convergenceThreshold)
{
    // Estimate convergence time based on eigenvalues
    // This is a simplified estimate for 2x2 systems

    if (stateSize == 2) {
        // Compute closed-loop dynamics: A - B*K
        float CL[4];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                CL[i * 2 + j] = A[i * 2 + j];
                for (int k = 0; k < controlSize; k++) {
                    CL[i * 2 + j] -= B[i * controlSize + k] * K[k * stateSize + j];
                }
            }
        }

        // Approximate dominant eigenvalue using trace and determinant
        float trace = CL[0] + CL[3];
        float det = CL[0] * CL[3] - CL[1] * CL[2];

        // Characteristic equation: λ² - trace·λ + det = 0
        float discriminant = trace * trace - 4 * det;

        if (discriminant >= 0) {
            // Real eigenvalues
            float lambda1 = (trace + sqrt(discriminant)) / 2;
            float lambda2 = (trace - sqrt(discriminant)) / 2;

            // Dominant eigenvalue (larger magnitude)
            float domEigenvalue = (fabs(lambda1) > fabs(lambda2)) ? lambda1 : lambda2;

            if (fabs(domEigenvalue) < 1.0f && fabs(domEigenvalue) > 0.0f) {
                // Estimate time constant
                float timeConstant = -1.0f / log(fabs(domEigenvalue));

                // Time to reach convergenceThreshold
                return timeConstant * log(1.0f / convergenceThreshold);
            }
        }
    }

    // Default value if calculation fails
    return -1.0f;
}

bool AutoLQR::exportGains(float* exportedK)
{
    if (!K || !exportedK)
        return false;

    matrixCopy(K, exportedK, controlSize * stateSize);
    return true;
}

float AutoLQR::calculateExpectedCost()
{
    if (!P || !state)
        return -1.0f;

    // Cost = x'Px
    float cost = 0;
    for (int i = 0; i < stateSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            cost += state[i] * P[i * stateSize + j] * state[j];
        }
    }

    return cost;
}

bool AutoLQR::computeGainMatrixIterative()
{
    // Método Iterativo de Riccati com Warm Start para DARE
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Verificar warm start
    bool has_warm_start = false;
    float P_norm = 0.0f;
    for (int i = 0; i < stateSize * stateSize; i++) {
        P_norm += fabsf(P[i]);
    }
    has_warm_start = (P_norm > 1e-6f);

    // Se não tem warm start, inicializa P = Q
    if (!has_warm_start) {
        matrixCopy(Q, P, stateSize * stateSize);
    }

    // Pré-alocação de memória
    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int nm = n * m;
    const int mm = m * m;
    
    float* P_new = new float[nn]();
    float* AT = new float[nn]();
    float* BT = new float[nm]();
    float* PA = new float[nn]();
    float* PB = new float[nm]();
    float* ATPA = new float[nn]();
    float* BTPB = new float[mm]();
    float* BTPA = new float[m * n]();
    float* S = new float[mm]();
    float* S_inv = new float[mm]();
    float* K_temp = new float[m * n]();
    float* ATPB = new float[nm]();
    float* correction = new float[nn]();

    // Calcular transpostas (constantes)
    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);

    const int maxIterations = 200;
    const float tolerance = 1e-6f;
    bool converged = false;

    for (int iter = 0; iter < maxIterations; iter++) {
        // ================================================================
        // ITERAÇÃO DARE
        // P_new = Q + A'PA - A'PB(R + B'PB)^{-1}B'PA
        // ================================================================

        matrixMultiply(P, A, PA, n, n, n);
        matrixMultiply(P, B, PB, n, n, m);
        matrixMultiply(AT, PA, ATPA, n, n, n);
        matrixMultiply(BT, PB, BTPB, m, n, m);
        matrixMultiply(BT, PA, BTPA, m, n, n);
        
        matrixAdd(R, BTPB, S, m, m);
        
        // Regularização
        for (int i = 0; i < m; i++) {
            S[i * m + i] += 1e-8f;
        }
        
        matrixCopy(S, S_inv, mm);
        if (!invertMatrix(S_inv, S_inv, m)) {
            break;
        }
        
        matrixMultiply(S_inv, BTPA, K_temp, m, m, n);
        matrixMultiply(AT, PB, ATPB, n, n, m);
        matrixMultiply(ATPB, K_temp, correction, n, m, n);
        
        float diff = 0.0f;
        float P_new_norm = 0.0f;
        
        for (int i = 0; i < nn; i++) {
            P_new[i] = Q[i] + ATPA[i] - correction[i];
            diff += fabsf(P_new[i] - P[i]);
            P_new_norm += fabsf(P_new[i]);
        }
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P_new[i * n + j] + P_new[j * n + i]) * 0.5f;
                P_new[i * n + j] = avg;
                P_new[j * n + i] = avg;
            }
        }
        
        matrixCopy(P_new, P, nn);
        
        float rel_diff = (P_new_norm > 1e-10f) ? (diff / P_new_norm) : diff;
        
        if (rel_diff < tolerance) {
            converged = true;
            break;
        }
    }

    // ================================================================
    // CÁLCULO FINAL DO GANHO K
    // ================================================================
    matrixMultiply(P, A, PA, n, n, n);
    matrixMultiply(P, B, PB, n, n, m);
    matrixMultiply(BT, PB, BTPB, m, n, m);
    matrixMultiply(BT, PA, BTPA, m, n, n);
    
    matrixAdd(R, BTPB, S, m, m);
    for (int i = 0; i < m; i++) {
        S[i * m + i] += 1e-8f;
    }
    
    matrixCopy(S, S_inv, mm);
    if (invertMatrix(S_inv, S_inv, m)) {
        matrixMultiply(S_inv, BTPA, K, m, m, n);
    } else {
        converged = false;
    }

    // Limpeza
    delete[] P_new;
    delete[] AT;
    delete[] BT;
    delete[] PA;
    delete[] PB;
    delete[] ATPA;
    delete[] BTPB;
    delete[] BTPA;
    delete[] S;
    delete[] S_inv;
    delete[] K_temp;
    delete[] ATPB;
    delete[] correction;

    return converged;
}
