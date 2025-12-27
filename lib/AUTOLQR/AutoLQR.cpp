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
    } else if (strcmp(method, "SDA_SS") == 0) {
        K_flag = computeGainMatrixSDA_SS();
    } else if (strcmp(method, "ASDA") == 0) {
        K_flag = computeGainMatrixASDA();
    } else if (strcmp(method, "SDA_SCALED") == 0) {
        K_flag = computeGainMatrixSDA_Scaled();
    } else if (strcmp(method, "SCHUR") == 0) {
        K_flag = computeGainMatrixSchur();
    } else if (strcmp(method, "VAN_DOOREN") == 0) {
        K_flag = computeGainMatrixVanDooren();
    } else if (strcmp(method, "ITERATIVE") == 0) {
        K_flag = computeGainMatrixIterative();
    } else if (strcmp(method, "ADDA") == 0) {
        K_flag = computeGainMatrixADDA();
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
    const float tolerance = 1e-6f;
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
    // Método de Schur para DARE usando formulação do pencil simplético
    // Resolve: A'PA - P - A'PB(R+B'PB)^{-1}B'PA + Q = 0
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    
    // ========================================================================
    // PASSO 1: Alocar memória
    // ========================================================================
    float* R_inv = new float[m * m];
    float* BT = new float[m * n];
    float* AT = new float[n * n];
    float* A_inv = new float[n * n];
    float* G = new float[n * n];
    float* temp_nm = new float[n * m];
    float* temp_nn = new float[n * n];
    float* temp_nn2 = new float[n * n];
    
    // ========================================================================
    // PASSO 2: Calcular matrizes auxiliares
    // ========================================================================
    // R_inv = inv(R)
    matrixCopy(R, R_inv, m * m);
    if (!invertMatrix(R_inv, R_inv, m)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        return false;
    }
    
    // BT = B'
    transposeMatrix(B, BT, n, m);
    
    // AT = A'
    transposeMatrix(A, AT, n, n);
    
    // G = B * R^{-1} * B'
    matrixMultiply(B, R_inv, temp_nm, n, m, m);
    matrixMultiply(temp_nm, BT, G, n, m, n);
    
    // A_inv = inv(A)
    matrixCopy(A, A_inv, n * n);
    if (!invertMatrix(A_inv, A_inv, n)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        return false;
    }
    
    // ========================================================================
    // PASSO 3: Construir matriz Hamiltoniana simplética Z (2n x 2n)
    // ========================================================================
    // Para DARE, usamos a matriz simplética:
    // Z = [A + G*A'^{-1}*Q,  -G*A'^{-1}    ]
    //     [-A'^{-1}*Q,        A'^{-1}      ]
    // Os autovalores estáveis de Z dão a solução
    
    Eigen::MatrixXf Z(2*n, 2*n);
    
    // Calcular A'^{-1}
    float* AT_inv = new float[n * n];
    matrixCopy(AT, AT_inv, n * n);
    if (!invertMatrix(AT_inv, AT_inv, n)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        return false;
    }
    
    // Bloco (0,0): A + G*A'^{-1}*Q
    // temp_nn = A'^{-1} * Q
    matrixMultiply(AT_inv, Q, temp_nn, n, n, n);
    // temp_nn2 = G * (A'^{-1} * Q)
    matrixMultiply(G, temp_nn, temp_nn2, n, n, n);
    // Z(0:n, 0:n) = A + G*A'^{-1}*Q
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(i, j) = A[i * n + j] + temp_nn2[i * n + j];
        }
    }
    
    // Bloco (0,n): -G*A'^{-1}
    matrixMultiply(G, AT_inv, temp_nn, n, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(i, n + j) = -temp_nn[i * n + j];
        }
    }
    
    // Bloco (n,0): -A'^{-1}*Q
    matrixMultiply(AT_inv, Q, temp_nn, n, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(n + i, j) = -temp_nn[i * n + j];
        }
    }
    
    // Bloco (n,n): A'^{-1}
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(n + i, n + j) = AT_inv[i * n + j];
        }
    }
    
    // ========================================================================
    // PASSO 4: Decomposição de Schur complexa
    // ========================================================================
    Eigen::ComplexSchur<Eigen::MatrixXf> schur(Z);
    
    if (schur.info() != Eigen::Success) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        return false;
    }
    
    Eigen::MatrixXcf T = schur.matrixT();
    Eigen::MatrixXcf U = schur.matrixU();
    
    // ========================================================================
    // PASSO 5: Identificar autovalores estáveis (|λ| < 1)
    // ========================================================================
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    for (int i = 0; i < 2*n; i++) {
        std::complex<float> eigenvalue = T(i, i);
        float magnitude = std::abs(eigenvalue);
        
        if (magnitude < 1.0f && magnitude > 1e-10f) {
            stable_indices.push_back(i);
        }
    }
    
    // Se não encontrou n autovalores estáveis, tentar com threshold mais relaxado
    if (stable_indices.size() < static_cast<size_t>(n)) {
        stable_indices.clear();
        for (int i = 0; i < 2*n; i++) {
            std::complex<float> eigenvalue = T(i, i);
            float magnitude = std::abs(eigenvalue);
            
            if (magnitude < 1.0f + 1e-6f) {
                stable_indices.push_back(i);
                if (stable_indices.size() == static_cast<size_t>(n)) break;
            }
        }
    }
    
    if (stable_indices.size() != static_cast<size_t>(n)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        return false;
    }
    
    // ========================================================================
    // PASSO 6: Extrair subespaço invariante estável
    // ========================================================================
    Eigen::MatrixXcf U11(n, n);
    Eigen::MatrixXcf U21(n, n);
    
    for (int j = 0; j < n; j++) {
        int idx = stable_indices[j];
        for (int i = 0; i < n; i++) {
            U11(i, j) = U(i, idx);
            U21(i, j) = U(n + i, idx);
        }
    }
    
    // ========================================================================
    // PASSO 7: Calcular P = U21 * inv(U11)
    // ========================================================================
    Eigen::MatrixXcf P_complex = U21 * U11.inverse();
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i * n + j] = P_complex(i, j).real();
        }
    }
    
    // Forçar simetria de P
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
            P[i * n + j] = avg;
            P[j * n + i] = avg;
        }
    }
    
    // ========================================================================
    // PASSO 8: Calcular K = (R + B'*P*B)^{-1} * B'*P*A
    // ========================================================================
    float* BT_P = new float[m * n];
    float* BT_P_B = new float[m * m];
    float* BT_P_A = new float[m * n];
    float* term = new float[m * m];
    
    matrixMultiply(BT, P, BT_P, m, n, n);
    matrixMultiply(BT_P, B, BT_P_B, m, n, m);
    matrixAdd(R, BT_P_B, term, m, m);
    
    if (!invertMatrix(term, term, m)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        delete[] BT_P; delete[] BT_P_B; delete[] BT_P_A; delete[] term;
        return false;
    }
    
    matrixMultiply(BT_P, A, BT_P_A, m, n, n);
    matrixMultiply(term, BT_P_A, K, m, m, n);

    // ========================================================================
    // Limpeza de memória
    // ========================================================================
    delete[] R_inv;
    delete[] BT;
    delete[] AT;
    delete[] A_inv;
    delete[] G;
    delete[] temp_nm;
    delete[] temp_nn;
    delete[] temp_nn2;
    delete[] AT_inv;
    delete[] BT_P;
    delete[] BT_P_B;
    delete[] BT_P_A;
    delete[] term;
    
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

    const int maxIterations = 5000;
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

// ============================================================================
// SDA COM SINGLE SHIFT (SDA-ss)
// Versão melhorada do SDA com parâmetro de shift para convergência aprimorada
// quando autovalores estão próximos de 1
// ============================================================================
bool AutoLQR::computeGainMatrixSDA_SS()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;
    const int maxIterations = 100;
    const float tolerance = 1e-8f;
    
    // Parâmetro de shift - γ ótimo para DARE é tipicamente 1 para sistemas discretos
    float gamma = 1.0f;
    bool converged = false;
    float prev_diff = 1e10f;
    
    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* W = new float[nn]();
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    
    // Inicialização
    matrixCopy(A, Ak, nn);
    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    // Calcular R_inv
    matrixCopy(R, R_inv, mm);
    bool init_ok = invertMatrix(R_inv, R_inv, m);
    
    if (init_ok) {
        // Gk = B * R^(-1) * B'
        float* B_Rinv = new float[n * m];
        matrixMultiply(B, R_inv, B_Rinv, n, m, m);
        matrixMultiply(B_Rinv, BT, Gk, n, m, n);
        delete[] B_Rinv;
        
        // Hk = Q
        matrixCopy(Q, Hk, nn);
        
        // Aplicar shift inicial
        for (int i = 0; i < nn; i++) {
            Gk[i] *= (gamma * gamma);
        }

        // Loop SDA com shift
        for (int iter = 0; iter < maxIterations; iter++) {
            matrixMultiply(Gk, Hk, Temp1, n, n, n);
            for (int i = 0; i < n; i++) {
                Temp1[i * n + i] += 1.0f;
            }
            
            matrixCopy(Temp1, W, nn);
            if (!invertMatrix(W, W, n)) {
                break;
            }
            
            matrixMultiply(Ak, W, Temp1, n, n, n);
            matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
            
            transposeMatrix(Ak, AT, n, n);
            matrixMultiply(Gk, AT, Temp2, n, n, n);
            matrixMultiply(Temp1, Temp2, Temp3, n, n, n);
            matrixAdd(Gk, Temp3, Gk_next, n, n);
            
            matrixMultiply(W, Ak, Temp2, n, n, n);
            matrixMultiply(Hk, Temp2, Temp3, n, n, n);
            matrixMultiply(AT, Temp3, Temp2, n, n, n);
            matrixAdd(Hk, Temp2, Hk_next, n, n);
            
            float diff = 0.0f;
            float norm_Hk = 0.0f;
            for (int i = 0; i < nn; i++) {
                diff += (Hk_next[i] - Hk[i]) * (Hk_next[i] - Hk[i]);
                norm_Hk += Hk[i] * Hk[i];
            }
            diff = sqrtf(diff);
            norm_Hk = sqrtf(norm_Hk);
            
            float rel_diff = (norm_Hk > 1e-10f) ? (diff / norm_Hk) : diff;
            
            // Shift adaptativo
            if (iter > 5 && rel_diff > 0.9f * prev_diff) {
                gamma *= 0.95f;
                for (int i = 0; i < nn; i++) {
                    Gk_next[i] *= 0.9025f;
                }
            }
            prev_diff = rel_diff;
            
            matrixCopy(Ak_next, Ak, nn);
            matrixCopy(Gk_next, Gk, nn);
            matrixCopy(Hk_next, Hk, nn);
            
            if (rel_diff < tolerance) {
                converged = true;
                break;
            }
        }

        // P = Hk
        matrixCopy(Hk, P, nn);
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
                P[i * n + j] = avg;
                P[j * n + i] = avg;
            }
        }

        // Cálculo do ganho K
        float* BT_P = new float[m * n];
        float* BT_P_B = new float[mm];
        float* BT_P_A = new float[m * n];
        float* R_plus_BTPB = new float[mm];
        
        matrixMultiply(BT, P, BT_P, m, n, n);
        matrixMultiply(BT_P, B, BT_P_B, m, n, m);
        matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
        
        if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
            converged = false;
        } else {
            matrixMultiply(BT_P, A, BT_P_A, m, n, n);
            matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
        }
        
        delete[] BT_P;
        delete[] BT_P_B;
        delete[] BT_P_A;
        delete[] R_plus_BTPB;
    }

    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;

    return converged;
}

// ============================================================================
// SDA ADAPTATIVO (ASDA)
// Usa escalonamento adaptativo durante iterações para melhor estabilidade
// ============================================================================
bool AutoLQR::computeGainMatrixASDA()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;
    const int maxIterations = 100;
    const float tolerance = 1e-8f;
    bool converged = false;
    
    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* W = new float[nn]();
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    
    // Fatores de escalonamento adaptativo
    float alpha_k = 1.0f;
    float beta_k = 1.0f;
    
    // Inicialização ASDA
    matrixCopy(A, Ak, nn);
    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    matrixCopy(R, R_inv, mm);
    bool init_ok = invertMatrix(R_inv, R_inv, m);
    
    if (init_ok) {
        float* B_Rinv = new float[n * m];
        matrixMultiply(B, R_inv, B_Rinv, n, m, m);
        matrixMultiply(B_Rinv, BT, Gk, n, m, n);
        delete[] B_Rinv;
        
        matrixCopy(Q, Hk, nn);
        
        // Cálculo do escalonamento ótimo inicial
        float norm_A = 0.0f, norm_G = 0.0f, norm_H = 0.0f;
        for (int i = 0; i < nn; i++) {
            norm_A += Ak[i] * Ak[i];
            norm_G += Gk[i] * Gk[i];
            norm_H += Hk[i] * Hk[i];
        }
        norm_A = sqrtf(norm_A);
        norm_G = sqrtf(norm_G);
        norm_H = sqrtf(norm_H);
        
        if (norm_A > 1e-10f) {
            alpha_k = 1.0f / norm_A;
        }
        if (norm_H > 1e-10f && norm_G > 1e-10f) {
            beta_k = sqrtf(norm_H / norm_G);
        }
        
        // Aplicar escalonamento inicial
        for (int i = 0; i < nn; i++) {
            Gk[i] *= (beta_k * beta_k);
        }

        // Loop ASDA
        for (int iter = 0; iter < maxIterations; iter++) {
            // Passo 1: Calcular escalonamento adaptativo
            float norm_Gk = 0.0f, norm_Hk = 0.0f;
            for (int i = 0; i < nn; i++) {
                norm_Gk += Gk[i] * Gk[i];
                norm_Hk += Hk[i] * Hk[i];
            }
            norm_Gk = sqrtf(norm_Gk);
            norm_Hk = sqrtf(norm_Hk);
            
            float scale_factor = 1.0f;
            if (norm_Gk > 1e-10f && norm_Hk > 1e-10f) {
                scale_factor = sqrtf(norm_Hk / norm_Gk);
                scale_factor = fminf(fmaxf(scale_factor, 0.1f), 10.0f);
            }
            
            for (int i = 0; i < nn; i++) {
                Gk[i] *= scale_factor;
                Hk[i] /= scale_factor;
            }
            
            // Passo 2: Iteração SDA padrão
            matrixMultiply(Gk, Hk, Temp1, n, n, n);
            for (int i = 0; i < n; i++) {
                Temp1[i * n + i] += 1.0f;
            }
            
            matrixCopy(Temp1, W, nn);
            if (!invertMatrix(W, W, n)) {
                break;
            }
            
            matrixMultiply(Ak, W, Temp1, n, n, n);
            matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
            
            transposeMatrix(Ak, AT, n, n);
            matrixMultiply(Gk, AT, Temp2, n, n, n);
            matrixMultiply(Temp1, Temp2, Temp3, n, n, n);
            matrixAdd(Gk, Temp3, Gk_next, n, n);
            
            matrixMultiply(W, Ak, Temp2, n, n, n);
            matrixMultiply(Hk, Temp2, Temp3, n, n, n);
            matrixMultiply(AT, Temp3, Temp2, n, n, n);
            matrixAdd(Hk, Temp2, Hk_next, n, n);
            
            float diff = 0.0f;
            float norm_H_new = 0.0f;
            for (int i = 0; i < nn; i++) {
                float d = Hk_next[i] - Hk[i];
                diff += d * d;
                norm_H_new += Hk_next[i] * Hk_next[i];
            }
            diff = sqrtf(diff);
            norm_H_new = sqrtf(norm_H_new);
            
            float rel_diff = (norm_H_new > 1e-10f) ? (diff / norm_H_new) : diff;
            
            matrixCopy(Ak_next, Ak, nn);
            matrixCopy(Gk_next, Gk, nn);
            matrixCopy(Hk_next, Hk, nn);
            
            if (rel_diff < tolerance) {
                converged = true;
                break;
            }
        }

        // P = Hk
        matrixCopy(Hk, P, nn);
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
                P[i * n + j] = avg;
                P[j * n + i] = avg;
            }
        }

        // Cálculo do ganho K
        float* BT_P = new float[m * n];
        float* BT_P_B = new float[mm];
        float* BT_P_A = new float[m * n];
        float* R_plus_BTPB = new float[mm];
        
        matrixMultiply(BT, P, BT_P, m, n, n);
        matrixMultiply(BT_P, B, BT_P_B, m, n, m);
        matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
        
        if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
            converged = false;
        } else {
            matrixMultiply(BT_P, A, BT_P_A, m, n, n);
            matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
        }
        
        delete[] BT_P;
        delete[] BT_P_B;
        delete[] BT_P_A;
        delete[] R_plus_BTPB;
    }

    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;

    return converged;
}

// ============================================================================
// SDA COM ESCALONAMENTO ÓTIMO (Scaled SDA)
// Usa escalonamento ótimo do pencil Hamiltoniano para melhor condicionamento
// ============================================================================
bool AutoLQR::computeGainMatrixSDA_Scaled()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;
    const int maxIterations = 100;
    const float tolerance = 1e-8f;
    bool converged = false;
    
    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* W = new float[nn]();
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    
    // Matrizes de escalonamento
    float* D = new float[n]();
    float* Dinv = new float[n]();
    
    // Calcular escalonamento diagonal baseado nas normas das linhas de A
    for (int i = 0; i < n; i++) {
        float row_norm = 0.0f;
        for (int j = 0; j < n; j++) {
            row_norm += A[i * n + j] * A[i * n + j];
        }
        row_norm = sqrtf(row_norm);
        D[i] = (row_norm > 1e-10f) ? (1.0f / sqrtf(row_norm)) : 1.0f;
        Dinv[i] = 1.0f / D[i];
    }
    
    // Aplicar escalonamento a A: A_scaled = D * A * D^(-1)
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Ak[i * n + j] = D[i] * A[i * n + j] * Dinv[j];
        }
    }
    
    transposeMatrix(Ak, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    // Calcular R_inv
    matrixCopy(R, R_inv, mm);
    bool init_ok = invertMatrix(R_inv, R_inv, m);
    
    if (init_ok) {
        // Calcular G com escalonamento
        float* B_scaled = new float[n * m];
        float* B_Rinv = new float[n * m];
        float* BT_scaled = new float[m * n];
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                B_scaled[i * m + j] = D[i] * B[i * m + j];
            }
        }
        
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                BT_scaled[i * n + j] = BT[i * n + j] * D[j];
            }
        }
        
        matrixMultiply(B_scaled, R_inv, B_Rinv, n, m, m);
        matrixMultiply(B_Rinv, BT_scaled, Gk, n, m, n);
        
        delete[] B_scaled;
        delete[] B_Rinv;
        delete[] BT_scaled;
        
        // Q_scaled = D * Q * D
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                Hk[i * n + j] = D[i] * Q[i * n + j] * D[j];
            }
        }

        // Loop SDA escalonado
        for (int iter = 0; iter < maxIterations; iter++) {
            matrixMultiply(Gk, Hk, Temp1, n, n, n);
            for (int i = 0; i < n; i++) {
                Temp1[i * n + i] += 1.0f;
            }
            
            matrixCopy(Temp1, W, nn);
            if (!invertMatrix(W, W, n)) {
                break;
            }
            
            matrixMultiply(Ak, W, Temp1, n, n, n);
            matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
            
            transposeMatrix(Ak, AT, n, n);
            matrixMultiply(Gk, AT, Temp2, n, n, n);
            matrixMultiply(Temp1, Temp2, Temp3, n, n, n);
            matrixAdd(Gk, Temp3, Gk_next, n, n);
            
            matrixMultiply(W, Ak, Temp2, n, n, n);
            matrixMultiply(Hk, Temp2, Temp3, n, n, n);
            matrixMultiply(AT, Temp3, Temp2, n, n, n);
            matrixAdd(Hk, Temp2, Hk_next, n, n);
            
            float diff = 0.0f;
            float norm_H = 0.0f;
            for (int i = 0; i < nn; i++) {
                float d = Hk_next[i] - Hk[i];
                diff += d * d;
                norm_H += Hk_next[i] * Hk_next[i];
            }
            diff = sqrtf(diff);
            norm_H = sqrtf(norm_H);
            
            float rel_diff = (norm_H > 1e-10f) ? (diff / norm_H) : diff;
            
            matrixCopy(Ak_next, Ak, nn);
            matrixCopy(Gk_next, Gk, nn);
            matrixCopy(Hk_next, Hk, nn);
            
            if (rel_diff < tolerance) {
                converged = true;
                break;
            }
        }

        // Recuperar P original: P = D^(-1) * P_scaled * D^(-1)
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                P[i * n + j] = Dinv[i] * Hk[i * n + j] * Dinv[j];
            }
        }
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
                P[i * n + j] = avg;
                P[j * n + i] = avg;
            }
        }

        // Cálculo do ganho K
        float* BT_P = new float[m * n];
        float* BT_P_B = new float[mm];
        float* BT_P_A = new float[m * n];
        float* R_plus_BTPB = new float[mm];
        
        matrixMultiply(BT, P, BT_P, m, n, n);
        matrixMultiply(BT_P, B, BT_P_B, m, n, m);
        matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
        
        if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
            converged = false;
        } else {
            matrixMultiply(BT_P, A, BT_P_A, m, n, n);
            matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
        }
        
        delete[] BT_P;
        delete[] BT_P_B;
        delete[] BT_P_A;
        delete[] R_plus_BTPB;
    }

    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
    delete[] D; delete[] Dinv;

    return converged;
}

bool AutoLQR::computeGainMatrixADDA()
{
    // Implementação do Alternating-Directional Doubling Algorithm (ADDA)
    // Baseado em: "A structure-preserving doubling algorithm for nonsymmetric
    // algebraic Riccati equation" - Lin, Xu (2006)
    // 
    // O ADDA usa duas matrizes auxiliares V e W calculadas de forma simétrica:
    //   V = (I + Gk·Hk)^(-1)
    //   W = (I + Hk·Gk)^(-1)
    // 
    // Iterações:
    //   Ak+1 = Ak·V·Ak = Ak·W·Ak  (equivalentes pela simetria)
    //   Gk+1 = Gk + Ak·V·Gk·Ak'
    //   Hk+1 = Hk + Ak'·Hk·V·Ak
    
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
    float* V = new float[stateSize * stateSize]();
    float* Temp1 = new float[stateSize * stateSize]();
    float* Temp2 = new float[stateSize * stateSize]();
    float* Temp3 = new float[stateSize * stateSize]();
    
    // ========================================================================
    // INICIALIZAÇÃO DO ADDA
    // ========================================================================
    
    // 1. Ak = A
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
        delete[] V; delete[] Temp1; delete[] Temp2; delete[] Temp3;
        return false;
    }
    
    // 4. Gk = B * R^(-1) * B'
    float* B_Rinv = new float[stateSize * controlSize];
    matrixMultiply(B, R_inv, B_Rinv, stateSize, controlSize, controlSize);
    matrixMultiply(B_Rinv, BT, Gk, stateSize, controlSize, stateSize);
    delete[] B_Rinv;

    // 5. Hk = Q
    matrixCopy(Q, Hk, stateSize * stateSize);

    // ========================================================================
    // LOOP ADDA - Iterações de dobramento
    // ========================================================================
    const int maxIterations = 5000;
    const float tolerance = 1e-6f;
    bool converged = false;

    for (int iter = 0; iter < maxIterations; iter++) {
        // V = (I + Gk·Hk)^(-1)
        matrixMultiply(Gk, Hk, Temp1, stateSize, stateSize, stateSize);
        
        for (int i = 0; i < stateSize; i++) {
            Temp1[i * stateSize + i] += 1.0f;
        }
        
        matrixCopy(Temp1, V, stateSize * stateSize);
        if (!invertMatrix(V, V, stateSize)) {
            break;
        }
        
        // ================================================================
        // Iterações ADDA (equivalentes ao SDA para DARE simétrica)
        // ================================================================
        
        // Temp1 = Ak·V
        matrixMultiply(Ak, V, Temp1, stateSize, stateSize, stateSize);
        
        // Ak_next = (Ak·V)·Ak
        matrixMultiply(Temp1, Ak, Ak_next, stateSize, stateSize, stateSize);
        
        // Gk_next = Gk + (Ak·V)·Gk·Ak'
        transposeMatrix(Ak, AT, stateSize, stateSize);
        matrixMultiply(Gk, AT, Temp2, stateSize, stateSize, stateSize);
        matrixMultiply(Temp1, Temp2, Temp3, stateSize, stateSize, stateSize);
        matrixAdd(Gk, Temp3, Gk_next, stateSize, stateSize);
        
        // Hk_next = Hk + Ak'·Hk·V·Ak
        matrixMultiply(V, Ak, Temp2, stateSize, stateSize, stateSize);
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
    delete[] V; delete[] Temp1; delete[] Temp2; delete[] Temp3;
    delete[] BT_P; delete[] BT_P_B; delete[] BT_P_A; delete[] R_plus_BTPB;

    return converged;
}
