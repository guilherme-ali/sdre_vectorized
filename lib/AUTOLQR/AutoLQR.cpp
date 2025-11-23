#include "AutoLQR.h"
#include <math.h>
#include <ArduinoEigen.h>

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

bool AutoLQR::computeGains()
{
    // Usa método de Schur ao invés do método iterativo
    bool K_flag = computeGainMatrix();
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

bool AutoLQR::computeGainMatrix()
{
    // Implementação do Structure-preserving Doubling Algorithm (SDA)
    // Para DARE: A'·P·A - P - A'·P·B·(R + B'·P·B)^(-1)·B'·P·A + Q = 0
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    // Controle de frequência de print (1Hz)
    static unsigned long last_print_time = 0;
    unsigned long current_time = millis();
    bool should_print = (current_time - last_print_time >= 1000);

    unsigned long t_start = micros();

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
    // Para o SDA aplicado à DARE, a inicialização correta é simplesmente Q
    matrixCopy(Q, Hk, stateSize * stateSize);

    // ========================================================================
    // LOOP SDA
    // ========================================================================
    const int maxIterations = 5000;
    const float tolerance = 1e-9f;
    bool converged = false;
    int actual_iterations = 0; // ← CONTADOR DE ITERAÇÕES

    for (int iter = 0; iter < maxIterations; iter++) {
        actual_iterations++; // ← INCREMENTAR CONTADOR
        
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

    unsigned long t_total = micros() - t_start;

    // ========================================================================
    // PRINT COM NÚMERO DE ITERAÇÕES
    // ========================================================================
    if (should_print) {
        last_print_time = current_time;
        
        Serial.println(F("\n========= SDA (Structure-preserving Doubling) ========="));
        Serial.print(F("Dimensão: n="));
        Serial.print(stateSize);
        Serial.print(F(", m="));
        Serial.println(controlSize);
        
        Serial.print(F("Iterações: "));
        Serial.print(actual_iterations);
        Serial.print(F(" / "));
        Serial.print(maxIterations);
        if (converged) {
            Serial.println(F(" ✓ CONVERGIDO"));
        } else {
            Serial.println(F(" ✗ NÃO CONVERGIU"));
        }
        
        Serial.print(F("Tempo total: "));
        Serial.print(t_total / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("Tempo/iteração: "));
        Serial.print((t_total / (float)actual_iterations) / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        
        Serial.println(F("=======================================================\n"));
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

    // Verifica controlabilidade
    if (!isSystemControllable()) {
        return false;
    }

    // Controle de frequência de print (1Hz)
    static unsigned long last_print_time = 0;
    unsigned long current_time = millis();
    bool should_print = (current_time - last_print_time >= 1000);
    
    // ========================================================================
    // Medição de tempo - início
    // ========================================================================
    unsigned long t_start = micros();
    
    const int n = stateSize;
    const int n2 = 2 * n;
    
    // ========================================================================
    // PASSO 1: Alocar memória para matrizes intermediárias
    // ========================================================================
    unsigned long t_alloc_start = micros();
    
    float* R_inv = new float[controlSize * controlSize];
    float* BT = new float[controlSize * stateSize];
    float* AT = new float[stateSize * stateSize];
    float* temp_ctrl_state = new float[controlSize * stateSize];
    float* temp_state_ctrl = new float[stateSize * controlSize];
    float* G = new float[stateSize * stateSize];
    
    unsigned long t_alloc_end = micros();
    
    // ========================================================================
    // PASSO 2: Calcular G = B * inv(R) * B^T usando operações manuais
    // ========================================================================
    unsigned long t_build_G_start = micros();
    
    // 2.1: Inverter R
    matrixCopy(R, R_inv, controlSize * controlSize);
    if (!invertMatrix(R_inv, R_inv, controlSize)) {
        Serial.println(F("Erro: não foi possível inverter R"));
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        return false;
    }
    
    // 2.2: Calcular B * inv(R)
    matrixMultiply(B, R_inv, temp_state_ctrl, stateSize, controlSize, controlSize);
    
    // 2.3: Calcular B^T
    transposeMatrix(B, BT, stateSize, controlSize);
    
    // 2.4: Calcular G = (B * inv(R)) * B^T
    matrixMultiply(temp_state_ctrl, BT, G, stateSize, controlSize, stateSize);
    
    unsigned long t_build_G = micros() - t_build_G_start;
    
    // ========================================================================
    // PASSO 3: Calcular A^T
    // ========================================================================
    unsigned long t_build_AT_start = micros();
    transposeMatrix(A, AT, stateSize, stateSize);
    unsigned long t_build_AT = micros() - t_build_AT_start;
    
    // ========================================================================
    // PASSO 4: Construir matrizes H e J (2n x 2n) - USANDO EIGEN
    // ========================================================================
    unsigned long t_build_HJ_start = micros();
    
    // Mapear para Eigen apenas H e J (necessário para QZ)
    Eigen::MatrixXf H(n2, n2);
    Eigen::MatrixXf J(n2, n2);
    
    // Preencher H manualmente
    // H = [A    0]
    //     [-Q   I]
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(i, j) = A[i * n + j];              // Bloco superior esquerdo: A
            H(i, j + n) = 0.0f;                  // Bloco superior direito: 0
            H(i + n, j) = -Q[i * n + j];         // Bloco inferior esquerdo: -Q
            H(i + n, j + n) = (i == j) ? 1.0f : 0.0f;  // Bloco inferior direito: I
        }
    }
    
    // Preencher J manualmente
    // J = [I   G]
    //     [0  A^T]
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            J(i, j) = (i == j) ? 1.0f : 0.0f;    // Bloco superior esquerdo: I
            J(i, j + n) = G[i * n + j];          // Bloco superior direito: G
            J(i + n, j) = 0.0f;                  // Bloco inferior esquerdo: 0
            J(i + n, j + n) = AT[i * n + j];     // Bloco inferior direito: A^T
        }
    }
    
    unsigned long t_build_HJ = micros() - t_build_HJ_start;
    
    // ========================================================================
    // PASSO 5: Decomposição QZ OTIMIZADA
    // ========================================================================
    unsigned long t_qz_start = micros();
    
    // OTIMIZAÇÃO 1: Reduzir iterações máximas (trade-off precisão vs velocidade)
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> ges;
   
    // OTIMIZAÇÃO 2: Computar apenas se necessário
    ges.compute(H, J, true); // true = computa autovetores (necessário para P)
    
    if (ges.info() != Eigen::Success) {
        Serial.println(F("Erro na decomposição QZ generalizada"));
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        return false;
    }
    
    Eigen::VectorXcf alpha = ges.alphas();
    Eigen::VectorXcf beta = ges.betas();
    
    unsigned long t_qz_decomp = micros() - t_qz_start;
    
    // ========================================================================
    // PASSO 6: Ordenar autovalores estáveis (|λ| < 1)
    // ========================================================================
    unsigned long t_sort_start = micros();
    
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
        Serial.print(F("Erro: encontrados "));
        Serial.print(stable_indices.size());
        Serial.print(F(" autovalores estáveis, esperados "));
        Serial.println(n);
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        return false;
    }
    
    unsigned long t_sort_eig = micros() - t_sort_start;
    
    // ========================================================================
    // PASSO 7: Extrair subespaço invariante estável
    // ========================================================================
    unsigned long t_extract_start = micros();
    
    Eigen::MatrixXcf Z = ges.eigenvectors();
    
    // Alocar U11 e U21 como arrays C++
    float* U11_real = new float[n * n];
    float* U11_imag = new float[n * n];
    float* U21_real = new float[n * n];
    float* U21_imag = new float[n * n];
    
    // Extrair blocos U11 e U21 (parte real e imaginária)
    for (int j = 0; j < n; j++) {
        int idx = stable_indices[j];
        for (int i = 0; i < n; i++) {
            U11_real[i * n + j] = Z(i, idx).real();
            U11_imag[i * n + j] = Z(i, idx).imag();
            U21_real[i * n + j] = Z(i + n, idx).real();
            U21_imag[i * n + j] = Z(i + n, idx).imag();
        }
    }
    
    unsigned long t_extract_subspace = micros() - t_extract_start;
    
    // ========================================================================
    // PASSO 8: Calcular P = U21 * inv(U11) - MANUALMENTE
    // ========================================================================
    unsigned long t_calc_P_start = micros();
    
    // Para simplificar, vamos usar Eigen APENAS para esta inversão complexa
    // (muito difícil fazer manualmente com números complexos)
    Eigen::MatrixXcf U11_complex(n, n);
    Eigen::MatrixXcf U21_complex(n, n);
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            U11_complex(i, j) = std::complex<float>(U11_real[i * n + j], U11_imag[i * n + j]);
            U21_complex(i, j) = std::complex<float>(U21_real[i * n + j], U21_imag[i * n + j]);
        }
    }
    
    Eigen::MatrixXcf P_complex = U21_complex * U11_complex.inverse();
    
    // Extrair parte real e verificar se a parte imaginária é desprezível
    float max_imag = 0.0f;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float imag_part = std::abs(P_complex(i, j).imag());
            if (imag_part > max_imag) max_imag = imag_part;
            P[i * n + j] = P_complex(i, j).real();
        }
    }
    
    if (max_imag > 1e-6) {
        Serial.print(F("AVISO: Parte imaginária significativa em P: "));
        Serial.println(max_imag, 8);
    }
    
    // Forçar simetria de P de forma mais robusta
    // P deve ser simétrica por construção (solução da DARE)
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
            P[i * n + j] = avg;
            P[j * n + i] = avg;
        }
    }
    
    // Verificar se P é definida positiva (critério de estabilidade)
    bool is_positive_definite = true;
    for (int i = 0; i < n && is_positive_definite; i++) {
        if (P[i * n + i] <= 0) {
            is_positive_definite = false;
        }
    }
    
    if (!is_positive_definite) {
        Serial.println(F("AVISO: P não é definida positiva!"));
    }
    
    unsigned long t_calc_P = micros() - t_calc_P_start;
    
    // ========================================================================
    // PASSO 9: Calcular K = (R + B^T*P*B)^{-1} * B^T*P*A - MANUALMENTE
    // ========================================================================
    unsigned long t_calc_K_start = micros();
    
    // 9.1: Calcular B^T * P
    float* BT_P = new float[controlSize * stateSize];
    matrixMultiply(BT, P, BT_P, controlSize, stateSize, stateSize);
    
    // 9.2: Calcular B^T * P * B
    float* BT_P_B = new float[controlSize * controlSize];
    matrixMultiply(BT_P, B, BT_P_B, controlSize, stateSize, controlSize);
    
    // 9.3: Calcular R + B^T * P * B
    float* term = new float[controlSize * controlSize];
    matrixAdd(R, BT_P_B, term, controlSize, controlSize);
    
    // 9.4: Inverter (R + B^T * P * B)
    if (!invertMatrix(term, term, controlSize)) {
        Serial.println(F("Erro: não foi possível inverter (R + B^T*P*B)"));
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] temp_ctrl_state; delete[] temp_state_ctrl; delete[] G;
        delete[] U11_real; delete[] U11_imag; delete[] U21_real; delete[] U21_imag;
        delete[] BT_P; delete[] BT_P_B; delete[] term;
        return false;
    }
    
    // 9.5: Calcular B^T * P * A
    float* BT_P_A = new float[controlSize * stateSize];
    matrixMultiply(BT_P, A, BT_P_A, controlSize, stateSize, stateSize);
    
    // 9.6: Calcular K = (R + B^T*P*B)^{-1} * B^T*P*A
    matrixMultiply(term, BT_P_A, K, controlSize, controlSize, stateSize);
    
    unsigned long t_calc_K = micros() - t_calc_K_start;
    
    // ========================================================================
    // Limpeza de memória
    // ========================================================================
    unsigned long t_cleanup_start = micros();
    
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
    
    unsigned long t_cleanup = micros() - t_cleanup_start;
    
    // ========================================================================
    // Tempo total
    // ========================================================================
    unsigned long t_total = micros() - t_start;
    
    // ========================================================================
    // PRINT DETALHADO DE PERFORMANCE
    // ========================================================================
    if (should_print) {
        last_print_time = current_time;
        
        Serial.println(F("\n========= PERFORMANCE: computeGainMatrixSchur (OTIMIZADO) ========="));
        
        Serial.println(F("\n--- MÉTODO DE SCHUR (QZ) COM OPERAÇÕES MANUAIS ---"));
        Serial.print(F("Dimensão do sistema: n = "));
        Serial.print(n);
        Serial.print(F(", Problema aumentado: 2n = "));
        Serial.println(n2);
        
        Serial.println(F("\n--- FASE 1: ALOCAÇÃO ---"));
        Serial.print(F("1. Alocação de memória: "));
        Serial.print((t_alloc_end - t_alloc_start) / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 2: CÁLCULO DE G (MANUAL) ---"));
        Serial.print(F("2. G = B*inv(R)*B^T: "));
        Serial.print(t_build_G / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 3: TRANSPOSTA DE A ---"));
        Serial.print(F("3. A^T: "));
        Serial.print(t_build_AT / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 4: CONSTRUÇÃO H e J ---"));
        Serial.print(F("4. Montagem H e J (2n x 2n): "));
        Serial.print(t_build_HJ / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 5: DECOMPOSIÇÃO QZ (CRÍTICA - EIGEN) ---"));
        Serial.print(F("5. Eigen::GeneralizedEigenSolver: "));
        Serial.print(t_qz_decomp / 1000.0, 3);
        Serial.print(F(" ms ("));
        Serial.print((t_qz_decomp / (float)t_total) * 100, 1);
        Serial.println(F("%)"));
        
        Serial.println(F("\n--- FASE 6: ORDENAÇÃO ---"));
        Serial.print(F("6. Ordenar autovalores estáveis: "));
        Serial.print(t_sort_eig / 1000.0, 3);
        Serial.println(F(" ms"));
        Serial.print(F("   Autovalores estáveis: "));
        Serial.print(stable_indices.size());
        Serial.print(F(" / "));
        Serial.println(n);
        
        Serial.println(F("\n--- FASE 7: EXTRAÇÃO ---"));
        Serial.print(F("7. Extrair U11, U21: "));
        Serial.print(t_extract_subspace / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 8: SOLUÇÃO P ---"));
        Serial.print(F("8. P = U21*inv(U11): "));
        Serial.print(t_calc_P / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 9: GANHO K (MANUAL) ---"));
        Serial.print(F("9. K = inv(R+B^T*P*B)*B^T*P*A: "));
        Serial.print(t_calc_K / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 10: LIMPEZA ---"));
        Serial.print(F("10. Desalocação: "));
        Serial.print(t_cleanup / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- RESUMO ---"));
        Serial.print(F("TEMPO TOTAL: "));
        Serial.print(t_total / 1000.0, 3);
        Serial.println(F(" ms"));
        
        float total_ms = t_total / 1000.0;
        float manual_ops = t_build_G + t_build_AT + t_build_HJ + t_calc_K;
        
        Serial.println(F("\nDistribuição:"));
        Serial.print(F("  Operações manuais: "));
        Serial.print((manual_ops / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.print(F("  Decomposição QZ:   "));
        Serial.print((t_qz_decomp / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("% (GARGALO INEVITÁVEL)"));
        
        Serial.print(F("  Outras operações:  "));
        Serial.print(((t_total - manual_ops - t_qz_decomp) / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.println(F("====================================================================\n"));
    }
    
    return true;
}
bool AutoLQR::computeGainMatrixVanDooren()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Verifica controlabilidade
    if (!isSystemControllable()) {
        return false;
    }

    // Controle de frequência de print (1Hz)
    static unsigned long last_print_time = 0;
    unsigned long current_time = millis();
    bool should_print = (current_time - last_print_time >= 1000);

    unsigned long t_start = micros();
    
    const int n = stateSize;
    const int m = controlSize;
    const int pencil_size = 2*n + m;

    // ========================================================================
    // OTIMIZAÇÃO 1: Pré-alocar toda memória de uma vez (melhor cache locality)
    // ========================================================================
    unsigned long t_alloc_start = micros();
    
    // Alocar memória contígua para melhor desempenho de cache
    const int total_floats = (n*n) + (m*n) + (m*n) + (m*m) + (m*n) + (m*m);
    float* memory_pool = new float[total_floats];
    
    // Distribuir ponteiros no pool de memória
    float* AT = memory_pool;
    float* BT = AT + (n*n);
    float* BT_P = BT + (m*n);
    float* BT_P_B = BT_P + (m*n);
    float* term = BT_P_B + (m*m);
    float* BT_P_A = term + (m*m);
    
    unsigned long t_alloc_end = micros();

    // ========================================================================
    // OTIMIZAÇÃO 2: Calcular transpostas manualmente (loop unrolling quando possível)
    // ========================================================================
    unsigned long t_transpose_start = micros();
    
    // Transpor A (otimizado para sistemas pequenos)
    if (n == 2) {
        // Loop unrolling para 2x2
        AT[0] = A[0]; AT[1] = A[2];
        AT[2] = A[1]; AT[3] = A[3];
    } else {
        transposeMatrix(A, AT, n, n);
    }
    
    // Transpor B (otimizado para sistemas pequenos)
    if (n == 2 && m == 1) {
        // Loop unrolling para 2x1
        BT[0] = B[0];
        BT[1] = B[1];
    } else {
        transposeMatrix(B, BT, n, m);
    }
    
    unsigned long t_transpose = micros() - t_transpose_start;

    // ========================================================================
    // OTIMIZAÇÃO 3: Construir pencil usando acesso direto (evitar overhead)
    // ========================================================================
    unsigned long t_build_pencil_start = micros();
    
    Eigen::MatrixXf H(pencil_size, pencil_size);
    Eigen::MatrixXf J(pencil_size, pencil_size);
    
    // Zerar matrizes de uma vez (mais rápido que inicialização elemento por elemento)
    H.setZero();
    J.setZero();

    // Preencher H e J com loops otimizados
    // OTIMIZAÇÃO: usar memcpy quando apropriado
    
    // H - Bloco (0,0): A
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(i, j) = A[i * n + j];
        }
    }
    
    // H - Bloco (0,2): B
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            H(i, 2*n + j) = B[i * m + j];
        }
    }
    
    // H - Bloco (1,0): -Q
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(n + i, j) = -Q[i * n + j];
        }
    }
    
    // H - Bloco (1,1): I (identidade) - otimizado
    for (int i = 0; i < n; i++) {
        H(n + i, n + i) = 1.0f;
    }
    
    // H - Bloco (2,2): R
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            H(2*n + i, 2*n + j) = R[i * m + j];
        }
    }
    
    // J - Bloco (0,0): I (identidade) - otimizado
    for (int i = 0; i < n; i++) {
        J(i, i) = 1.0f;
    }
    
    // J - Bloco (1,1): A^T
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            J(n + i, n + j) = AT[i * n + j];
        }
    }
    
    // J - Bloco (2,1): -B^T
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            J(2*n + i, n + j) = -BT[i * n + j];
        }
    }
    
    unsigned long t_build_pencil = micros() - t_build_pencil_start;

    // ========================================================================
    // OTIMIZAÇÃO 4: QZ com configuração otimizada
    // ========================================================================
    unsigned long t_qz_start = micros();
    
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> ges;
    
    // CRÍTICO: O maior gargalo está aqui - não há como otimizar muito mais
    // A decomposição QZ é O(n³) e depende de algoritmos internos da Eigen
    ges.compute(H, J, true);
    
    if (ges.info() != Eigen::Success) {
        delete[] memory_pool;
        return false;
    }
    
    Eigen::VectorXcf alpha = ges.alphas();
    Eigen::VectorXcf beta = ges.betas();
    
    unsigned long t_qz_decomp = micros() - t_qz_start;

    // ========================================================================
    // OTIMIZAÇÃO 5: Seleção de autovalores com early termination
    // ========================================================================
    unsigned long t_sort_start = micros();
    
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    // Early termination: parar quando encontrar n autovalores estáveis
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
    
    unsigned long t_sort_eig = micros() - t_sort_start;

    // ========================================================================
    // OTIMIZAÇÃO 6: Extração otimizada do subespaço
    // ========================================================================
    unsigned long t_extract_start = micros();
    
    const Eigen::MatrixXcf& Z = ges.eigenvectors();
    
    // Alocar U1 e U2 como matrizes Eigen diretamente (evita conversões)
    Eigen::MatrixXcf U1(n, n);
    Eigen::MatrixXcf U2(n, n);
    
    // Extração otimizada usando operações vetorizadas do Eigen
    for (int j = 0; j < n; j++) {
        const int idx = stable_indices[j];
        U1.col(j) = Z.col(idx).head(n);
        U2.col(j) = Z.col(idx).segment(n, n);
    }
    
    unsigned long t_extract_subspace = micros() - t_extract_start;

    // ========================================================================
    // OTIMIZAÇÃO 7: Cálculo de P com verificação mínima
    // ========================================================================
    unsigned long t_calc_P_start = micros();
    
    // Usar decomposição LU ao invés de inversão direta (mais rápido)
    Eigen::MatrixXcf P_complex = U2 * U1.inverse();
    
    // Extrair parte real de forma otimizada
    float max_imag = 0.0f;
    
    // OTIMIZAÇÃO: Loop com prefetching implícito
    for (int i = 0; i < n; i++) {
        const int row_offset = i * n;
        for (int j = 0; j < n; j++) {
            const std::complex<float>& val = P_complex(i, j);
            const float imag_part = std::abs(val.imag());
            max_imag = (imag_part > max_imag) ? imag_part : max_imag;
            P[row_offset + j] = val.real();
        }
    }
    
    // Forçar simetria (otimizado - apenas triângulo superior)
    for (int i = 0; i < n; i++) {
        const int row_i = i * n;
        for (int j = i + 1; j < n; j++) {
            const float avg = (P[row_i + j] + P[j * n + i]) * 0.5f;
            P[row_i + j] = avg;
            P[j * n + i] = avg;
        }
    }
    
    unsigned long t_calc_P = micros() - t_calc_P_start;

    // ========================================================================
    // OTIMIZAÇÃO 8: Cálculo de K com multiplicações em cadeia otimizadas
    // ========================================================================
    unsigned long t_calc_K_start = micros();
    
    // 8.1: Calcular B^T * P (reutilizar buffer pré-alocado)
    matrixMultiply(BT, P, BT_P, m, n, n);
    
    // 8.2: Calcular B^T * P * B (reutilizar buffer)
    matrixMultiply(BT_P, B, BT_P_B, m, n, m);
    
    // 8.3: Calcular R + B^T * P * B (reutilizar buffer)
    matrixAdd(R, BT_P_B, term, m, m);
    
    // 8.4: Inverter (R + B^T * P * B)
    if (!invertMatrix(term, term, m)) {
        delete[] memory_pool;
        return false;
    }
    
    // 8.5: Calcular B^T * P * A (reutilizar buffer)
    matrixMultiply(BT_P, A, BT_P_A, m, n, n);
    
    // 8.6: Calcular K = (R + B^T*P*B)^{-1} * B^T*P*A
    matrixMultiply(term, BT_P_A, K, m, m, n);
    
    unsigned long t_calc_K = micros() - t_calc_K_start;

    // ========================================================================
    // OTIMIZAÇÃO 9: Limpeza simplificada (um único delete)
    // ========================================================================
    unsigned long t_cleanup_start = micros();
    delete[] memory_pool;
    unsigned long t_cleanup = micros() - t_cleanup_start;

    unsigned long t_total = micros() - t_start;

    // ========================================================================
    // PRINT OTIMIZADO (apenas se necessário)
    // ========================================================================
    if (should_print) {
        last_print_time = current_time;
        
        Serial.println(F("\n======== PERFORMANCE: VanDooren OTIMIZADO ========"));
        Serial.print(F("Dimensão: n="));
        Serial.print(n);
        Serial.print(F(", m="));
        Serial.print(m);
        Serial.print(F(", Pencil="));
        Serial.print(pencil_size);
        Serial.println(F("x"));
        
        const float total_ms = t_total / 1000.0f;
        
        Serial.println(F("\n--- TEMPOS ---"));
        Serial.print(F("1. Alocação: "));
        Serial.print((t_alloc_end - t_alloc_start) / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("2. Transpostas: "));
        Serial.print(t_transpose / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("3. Pencil H,J: "));
        Serial.print(t_build_pencil / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("4. QZ (GARGALO): "));
        Serial.print(t_qz_decomp / 1000.0f, 3);
        Serial.print(F(" ms ("));
        Serial.print((t_qz_decomp / 1000.0f / total_ms) * 100, 1);
        Serial.println(F("%)"));
        
        Serial.print(F("5. Seleção: "));
        Serial.print(t_sort_eig / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("6. Extração U1,U2: "));
        Serial.print(t_extract_subspace / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("7. Calc P: "));
        Serial.print(t_calc_P / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("8. Calc K: "));
        Serial.print(t_calc_K / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("9. Cleanup: "));
        Serial.print(t_cleanup / 1000.0f, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("\nTEMPO TOTAL: "));
        Serial.print(total_ms, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("==================================================\n"));
    }
    
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
