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
    bool K_flag = computeGainMatrixSchur();
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
    // Controle de frequência de print (1Hz)
    static unsigned long last_print_time = 0;
    unsigned long current_time = millis();
    bool should_print = (current_time - last_print_time >= 1000);
    
    unsigned long t_start = micros();
    unsigned long t_alloc_start, t_alloc_end;
    unsigned long t_check_init, t_init_P, t_transpose_start, t_transpose_end, t_iter_start;
    unsigned long t_mult_total = 0, t_invert_total = 0, t_add_total = 0, t_conv_total = 0;
    unsigned long t_mult1 = 0, t_mult2 = 0, t_mult3 = 0, t_mult4 = 0, t_mult5 = 0, t_mult6 = 0, t_mult7 = 0;
    int iteration_count = 0;
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Check if system is controllable
    if (!isSystemControllable()) {
        return false;
    }

    // Alocação de memória
    t_alloc_start = micros();
    float* P_next = new float[stateSize * stateSize]();
    float* BT = new float[controlSize * stateSize]();
    float* AT = new float[stateSize * stateSize]();
    float* temp1 = new float[controlSize * stateSize]();
    float* temp2 = new float[controlSize * controlSize]();
    float* temp3 = new float[controlSize * stateSize]();
    float* temp4 = new float[stateSize * stateSize]();
    float* temp5 = new float[stateSize * stateSize]();
    t_alloc_end = micros();

    // Check if P has been initialized with non-zero values
    t_check_init = micros();
    bool isPInitialized = false;
    for (int i = 0; i < stateSize * stateSize; i++) {
        if (fabs(P[i]) > 1e-6) {
            isPInitialized = true;
            break;
        }
    }

    // Initialize P as Q only if P hasn't been calculated before
    t_init_P = micros();
    if (!isPInitialized) {
        matrixCopy(Q, P, stateSize * stateSize);
    }
    // Otherwise, keep using the previously calculated P

    // Compute B transpose and A transpose
    t_transpose_start = micros();
    transposeMatrix(B, BT, stateSize, controlSize);
    transposeMatrix(A, AT, stateSize, stateSize);
    t_transpose_end = micros();

    // Iterate to solve DARE: P = A'PA - A'PB(R + B'PB)^(-1)B'PA + Q
    const int maxIterations = 5000000;
    const float tolerance = 1e-3f;

    t_iter_start = micros();

    // Pré-calcular A'P uma vez por iteração para reusar
    float* ATP = temp4; // Reutilizar buffer

    for (int iter = 0; iter < maxIterations; iter++) {
        iteration_count++;
        unsigned long t_step_start, t_invert_start, t_add_start, t_conv_start;
        
        // Calcular A'P uma vez (será usado em múltiplas operações)
        t_step_start = micros();
        matrixMultiply(AT, P, ATP, stateSize, stateSize, stateSize);
        t_mult4 += (micros() - t_step_start);
        
        // temp1 = B'P
        t_step_start = micros();
        matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);
        t_mult1 += (micros() - t_step_start);

        // temp2 = B'PB
        t_step_start = micros();
        matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);
        t_mult2 += (micros() - t_step_start);
        
        // temp2 = R + B'PB
        t_add_start = micros();
        matrixAdd(R, temp2, temp2, controlSize, controlSize);
        t_add_total += (micros() - t_add_start);

        // Invert (R + B'PB)
        t_invert_start = micros();
        if (!invertMatrix(temp2, temp2, controlSize)) {
            // Clean up memory
            delete[] P_next;
            delete[] BT;
            delete[] AT;
            delete[] temp1;
            delete[] temp2;
            delete[] temp3;
            delete[] temp4;
            delete[] temp5;
            return false;
        }
        t_invert_total += (micros() - t_invert_start);

        // temp3 = (R + B'PB)^(-1) * B'P
        t_step_start = micros();
        matrixMultiply(temp2, temp1, temp3, controlSize, controlSize, stateSize);
        t_mult3 += (micros() - t_step_start);

        // P_next = A'PA (usando ATP já calculado)
        t_step_start = micros();
        matrixMultiply(ATP, A, P_next, stateSize, stateSize, stateSize);
        t_mult5 += (micros() - t_step_start);

        // temp5 = A'PB (usando ATP já calculado)
        t_step_start = micros();
        matrixMultiply(ATP, B, temp5, stateSize, stateSize, controlSize);
        t_mult_total += (micros() - t_step_start);

        // temp5 = A'PB * (R + B'PB)^(-1) * B'PA - pode ser feito em dois passos
        t_step_start = micros();
        matrixMultiply(temp5, temp3, ATP, stateSize, controlSize, stateSize); // Reusar ATP como buffer
        t_mult6 += (micros() - t_step_start);
        
        t_step_start = micros();
        matrixMultiply(ATP, A, temp5, stateSize, stateSize, stateSize);
        t_mult7 += (micros() - t_step_start);

        // P_next = A'PA - A'PB * (R + B'PB)^(-1) * B'PA + Q
        t_add_start = micros();
        matrixSubtract(P_next, temp5, P_next, stateSize, stateSize);
        matrixAdd(P_next, Q, P_next, stateSize, stateSize);
        t_add_total += (micros() - t_add_start);

        // Check convergence apenas a cada N iterações para economizar tempo
        // (verificação é cara devido ao loop sobre todos elementos)
        const int CHECK_INTERVAL = 10; // Verifica convergência a cada 10 iterações
        
        if (iter % CHECK_INTERVAL == 0 || iter < 20) {
            t_conv_start = micros();
            float diff = 0;
            
            // Otimização: early exit se diferença ficar muito grande
            bool converged = true;
            for (int i = 0; i < stateSize * stateSize; i++) {
                float elem_diff = fabsf(P_next[i] - P[i]);
                diff += elem_diff;
                
                // Early exit: se já divergiu muito, não precisa continuar calculando
                if (diff > tolerance * 10.0f * (i + 1)) {
                    converged = false;
                    break;
                }
            }
            
            t_conv_total += (micros() - t_conv_start);
            
            // Update P for next iteration
            matrixCopy(P_next, P, stateSize * stateSize);

            if (converged && diff < tolerance) {
                break; // Converged
            }
        } else {
            // Apenas copiar P sem verificar convergência
            matrixCopy(P_next, P, stateSize * stateSize);
        }
    }

    unsigned long t_final_k_start = micros();
    unsigned long t_k_mult1, t_k_mult2, t_k_add, t_k_invert, t_k_mult3, t_k_mult4;
    
    // Compute K = (R + B'PB)^(-1) * B'PA
    t_k_mult1 = micros();
    matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);
    t_k_mult1 = micros() - t_k_mult1;
    
    t_k_mult2 = micros();
    matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);
    t_k_mult2 = micros() - t_k_mult2;

    t_k_add = micros();
    matrixAdd(R, temp2, temp2, controlSize, controlSize);
    t_k_add = micros() - t_k_add;

    t_k_invert = micros();
    if (!invertMatrix(temp2, temp2, controlSize)) {
        // Clean up memory
        delete[] P_next;
        delete[] BT;
        delete[] AT;
        delete[] temp1;
        delete[] temp2;
        delete[] temp3;
        delete[] temp4;
        delete[] temp5;
        return false;
    }
    t_k_invert = micros() - t_k_invert;

    t_k_mult3 = micros();
    matrixMultiply(temp1, A, temp3, controlSize, stateSize, stateSize);
    t_k_mult3 = micros() - t_k_mult3;
    
    t_k_mult4 = micros();
    matrixMultiply(temp2, temp3, K, controlSize, controlSize, stateSize);
    t_k_mult4 = micros() - t_k_mult4;

    unsigned long t_cleanup_start = micros();
    
    // Clean up memory
    delete[] P_next;
    delete[] BT;
    delete[] AT;
    delete[] temp1;
    delete[] temp2;
    delete[] temp3;
    delete[] temp4;
    delete[] temp5;
    
    unsigned long t_cleanup_end = micros();
    unsigned long t_total = micros();

    // Calcular totais de tempo das operações de multiplicação
    unsigned long t_mult_total_calc = t_mult1 + t_mult2 + t_mult3 + t_mult4 + t_mult5 + t_mult6 + t_mult7 + t_mult_total;

    // Imprimir análise de desempenho DETALHADA (apenas a cada 1 segundo)
    if (should_print) {
        last_print_time = current_time;
        
        Serial.println(F("\n============== PERFORMANCE ANALYSIS: computeGainMatrix =============="));
    
    Serial.println(F("\n--- FASE 1: INICIALIZAÇÃO ---"));
    Serial.print(F("1. Alocação de memória: "));
    Serial.print((t_alloc_end - t_alloc_start) / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("2. Check P inicializado: "));
    Serial.print((t_init_P - t_check_init) / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("3. Inicializar P (se necessário): "));
    Serial.print((t_transpose_start - t_init_P) / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("4. Transpor matrizes A e B: "));
    Serial.print((t_transpose_end - t_transpose_start) / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.println(F("\n--- FASE 2: LOOP DARE (Iterativo) ---"));
    Serial.print(F("Número de iterações: "));
    Serial.println(iteration_count);
    
    float total_loop_time = (t_final_k_start - t_iter_start) / 1000.0;
    Serial.print(F("Tempo total do loop: "));
    Serial.print(total_loop_time, 3);
    Serial.println(F(" ms"));
    
    Serial.println(F("\nDetalhamento por operação (totais acumulados):"));
    Serial.print(F("  - B'P (mult 1): "));
    Serial.print(t_mult1 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult1 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - B'PB (mult 2): "));
    Serial.print(t_mult2 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult2 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - R + B'PB (add): "));
    Serial.print(t_add_total / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_add_total / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - Inversão matriz (CRITICAL): "));
    Serial.print(t_invert_total / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_invert_total / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - (R+B'PB)^-1 * B'P (mult 3): "));
    Serial.print(t_mult3 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult3 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - A'P (mult 4): "));
    Serial.print(t_mult4 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult4 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - A'PA (mult 5): "));
    Serial.print(t_mult5 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult5 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - A'PB (mult extra): "));
    Serial.print(t_mult_total / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult_total / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - temp * temp (mult 6): "));
    Serial.print(t_mult6 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult6 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - Final mult (mult 7): "));
    Serial.print(t_mult7 / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult7 / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("  - Convergence check: "));
    Serial.print(t_conv_total / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_conv_total / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.print(F("\nTotal multiplicações: "));
    Serial.print(t_mult_total_calc / 1000.0, 3);
    Serial.print(F(" ms ("));
    Serial.print((t_mult_total_calc / 1000.0 / total_loop_time) * 100, 1);
    Serial.println(F("%)"));
    
    Serial.println(F("\n--- FASE 3: CÁLCULO FINAL DE K ---"));
    Serial.print(F("  - B'P: "));
    Serial.print(t_k_mult1 / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("  - B'PB: "));
    Serial.print(t_k_mult2 / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("  - R + B'PB: "));
    Serial.print(t_k_add / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("  - Inversão: "));
    Serial.print(t_k_invert / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("  - B'PA: "));
    Serial.print(t_k_mult3 / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.print(F("  - K final: "));
    Serial.print(t_k_mult4 / 1000.0, 3);
    Serial.println(F(" ms"));
    
    float total_k_time = (t_k_mult1 + t_k_mult2 + t_k_add + t_k_invert + t_k_mult3 + t_k_mult4) / 1000.0;
    Serial.print(F("Total fase K: "));
    Serial.print(total_k_time, 3);
    Serial.println(F(" ms"));
    
    Serial.println(F("\n--- FASE 4: LIMPEZA ---"));
    Serial.print(F("Desalocação de memória: "));
    Serial.print((t_cleanup_end - t_cleanup_start) / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.println(F("\n--- RESUMO GERAL ---"));
    Serial.print(F("TEMPO TOTAL: "));
    Serial.print((t_total - t_start) / 1000.0, 3);
    Serial.println(F(" ms"));
    
    Serial.println(F("\nPorcentagem por fase:"));
    float init_time = (t_transpose_end - t_start) / 1000.0;
    float total_time = (t_total - t_start) / 1000.0;
    
    Serial.print(F("  Inicialização: "));
    Serial.print((init_time / total_time) * 100, 1);
    Serial.println(F("%"));
    
    Serial.print(F("  Loop DARE: "));
    Serial.print((total_loop_time / total_time) * 100, 1);
    Serial.println(F("%"));
    
    Serial.print(F("  Cálculo K: "));
    Serial.print((total_k_time / total_time) * 100, 1);
    Serial.println(F("%"));
    
    Serial.print(F("  Limpeza: "));
    Serial.print(((t_cleanup_end - t_cleanup_start) / 1000.0 / total_time) * 100, 1);
    Serial.println(F("%"));
    
    Serial.println(F("======================================================================\n"));
    } // Fim do if (should_print)

    return true;
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
    unsigned long t_alloc_start, t_alloc_end;
    unsigned long t_check_init, t_init_P, t_transpose_start, t_transpose_end, t_iter_start;
    unsigned long t_mult_total = 0, t_invert_total = 0, t_add_total = 0, t_conv_total = 0;
    unsigned long t_mult1 = 0, t_mult2 = 0, t_mult3 = 0, t_mult4 = 0, t_mult5 = 0, t_mult6 = 0, t_mult7 = 0;
    int iteration_count = 0;
    
    // ========================================================================
    // PASSO 0: Preparação - Mapeamento para Eigen
    // ========================================================================
    unsigned long t_step = micros();
    
    // Mapeia arrays para matrizes Eigen (sem cópia)
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        A_map(A, stateSize, stateSize);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        B_map(B, stateSize, controlSize);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        Q_map(Q, stateSize, stateSize);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        R_map(R, controlSize, controlSize);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        P_map(P, stateSize, stateSize);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        K_map(K, controlSize, stateSize);

    t_alloc_start = micros() - t_step;

    // ========================================================================
    // PASSO 1: Construir matrizes H e J para problema generalizado (2n x 2n)
    // ========================================================================
    // H * z = λ * J * z
    // onde autovalores λ com |λ| < 1 correspondem ao subespaço estável
    
    const int n = stateSize;
    const int n2 = 2 * n;
    
    // Calcula G = B * inv(R) * B^T
    t_step = micros();
    Eigen::MatrixXf R_inv = R_map.inverse();
    Eigen::MatrixXf G = B_map * R_inv * B_map.transpose();
    unsigned long t_build_G = micros() - t_step;
    
    // Monta matriz H (Hamiltoniana superior - 2n x 2n)
    // H = [A    0]
    //     [-Q   I]
    t_step = micros();
    Eigen::MatrixXf H(n2, n2);
    H.block(0, 0, n, n) = A_map;                           // Bloco superior esquerdo: A
    H.block(0, n, n, n).setZero();                         // Bloco superior direito: 0
    H.block(n, 0, n, n) = -Q_map;                          // Bloco inferior esquerdo: -Q
    H.block(n, n, n, n).setIdentity();                     // Bloco inferior direito: I
    unsigned long t_build_H = micros() - t_step;
    
    // Monta matriz J (Hamiltoniana inferior - 2n x 2n)
    // J = [I   G]
    //     [0  A^T]
    t_step = micros();
    Eigen::MatrixXf J(n2, n2);
    J.block(0, 0, n, n).setIdentity();                     // Bloco superior esquerdo: I
    J.block(0, n, n, n) = G;                               // Bloco superior direito: G
    J.block(n, 0, n, n).setZero();                         // Bloco inferior esquerdo: 0
    J.block(n, n, n, n) = A_map.transpose();               // Bloco inferior direito: A^T
    unsigned long t_build_J = micros() - t_step;
    
    // ========================================================================
    // PASSO 2: Decomposição QZ Generalizada (Schur Generalizado)
    // ========================================================================
    // Resolve o problema de autovalores generalizado: H*v = λ*J*v
    // Eigen usa RealQZ ou ComplexQZ dependendo do tipo
    
    t_step = micros();
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> ges(H, J);
    
    if (ges.info() != Eigen::Success) {
        Serial.println(F("Erro na decomposição QZ generalizada"));
        return false;
    }
    
    // Autovalores: lambda = alpha / beta
    Eigen::VectorXcf alpha = ges.alphas();
    Eigen::VectorXcf beta = ges.betas();
    unsigned long t_qz_decomp = micros() - t_step;
    
    // ========================================================================
    // PASSO 3: Ordenar autovalores estáveis (|λ| < 1) - "Inside Unit Circle"
    // ========================================================================
    
    t_step = micros();
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    for (int i = 0; i < n2; i++) {
        // Evita divisão por zero
        if (std::abs(beta(i)) > 1e-10f) {
            std::complex<float> eigenvalue = alpha(i) / beta(i);
            float magnitude = std::abs(eigenvalue);
            
            // Seleciona autovalores dentro do círculo unitário (estáveis)
            if (magnitude < 1.0f) {
                stable_indices.push_back(i);
            }
        }
    }
    
    // Verifica se temos exatamente n autovalores estáveis
    if (stable_indices.size() != static_cast<size_t>(n)) {
        Serial.print(F("Erro: encontrados "));
        Serial.print(stable_indices.size());
        Serial.print(F(" autovalores estáveis, esperados "));
        Serial.println(n);
        return false;
    }
    unsigned long t_sort_eig = micros() - t_step;
    
    // ========================================================================
    // PASSO 4: Extrair subespaço invariante estável
    // ========================================================================
    // Matriz de autovetores Z (2n x 2n)
    t_step = micros();
    Eigen::MatrixXcf Z = ges.eigenvectors();
    
    // Particiona Z em blocos n x n, usando apenas colunas correspondentes
    // aos autovalores estáveis ordenados
    // Z = [U11  U12]
    //     [U21  U22]
    
    Eigen::MatrixXcf U11(n, n);
    Eigen::MatrixXcf U21(n, n);
    
    for (int j = 0; j < n; j++) {
        int idx = stable_indices[j];
        U11.col(j) = Z.block(0, idx, n, 1);  // Bloco superior
        U21.col(j) = Z.block(n, idx, n, 1);  // Bloco inferior
    }
    unsigned long t_extract_subspace = micros() - t_step;
    
    // ========================================================================
    // PASSO 5: Calcular solução P da DARE
    // ========================================================================
    // P = U21 * inv(U11)
    
    t_step = micros();
    Eigen::MatrixXcf P_complex = U21 * U11.inverse();
    
    // Converte para real (a solução deve ser real para sistema real)
    // Pega apenas a parte real e simetriza
    Eigen::MatrixXf P_result = P_complex.real();
    P_result = (P_result + P_result.transpose()) / 2.0f;
    
    // Copia resultado para array P
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P_map(i, j) = P_result(i, j);
        }
    }
    unsigned long t_calc_P = micros() - t_step;
    
    // ========================================================================
    // PASSO 6: Calcular ganho de realimentação K
    // ========================================================================
    // K = (R + B^T*P*B)^{-1} * B^T*P*A
    
    t_step = micros();
    Eigen::MatrixXf BT_P = B_map.transpose() * P_map;
    Eigen::MatrixXf term = R_map + BT_P * B_map;
    Eigen::MatrixXf K_result = term.inverse() * BT_P * A_map;
    
    // Copia resultado para array K
    for (int i = 0; i < controlSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            K_map(i, j) = K_result(i, j);
        }
    }
    unsigned long t_calc_K = micros() - t_step;
    
    // ========================================================================
    // Verificação: autovalores do sistema em malha fechada
    // ========================================================================
    t_step = micros();
    Eigen::MatrixXf A_cl = A_map - B_map * K_map;
    Eigen::EigenSolver<Eigen::MatrixXf> es(A_cl);
    
    bool stable = true;
    for (int i = 0; i < n; i++) {
        std::complex<float> eig = es.eigenvalues()(i);
        if (std::abs(eig) >= 1.0f) {
            stable = false;
            break;
        }
    }
    
    if (!stable) {
        Serial.println(F("Aviso: sistema em malha fechada pode ser instável"));
    }
    unsigned long t_verify = micros() - t_step;
    
    // ========================================================================
    // Tempo total
    // ========================================================================
    unsigned long t_total = micros() - t_start;
    
    // ========================================================================
    // PRINT DETALHADO DE PERFORMANCE (a cada 1 segundo)
    // ========================================================================
    if (should_print) {
        last_print_time = current_time;
        
        Serial.println(F("\n============ PERFORMANCE ANALYSIS: computeGainMatrixSchur ============"));
        
        Serial.println(F("\n--- MÉTODO DE SCHUR (QZ Generalizado) ---"));
        Serial.print(F("Dimensão do sistema: n = "));
        Serial.print(n);
        Serial.print(F(", Problema aumentado: 2n = "));
        Serial.println(n2);
        
        Serial.println(F("\n--- FASE 1: PREPARAÇÃO ---"));
        Serial.print(F("1. Mapeamento Eigen (sem cópia): "));
        Serial.print(t_alloc_start / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 2: CONSTRUÇÃO DAS HAMILTONIANAS ---"));
        Serial.print(F("2. Cálculo de G = B*inv(R)*B^T: "));
        Serial.print(t_build_G / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("3. Montagem matriz H (2n x 2n): "));
        Serial.print(t_build_H / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.print(F("4. Montagem matriz J (2n x 2n): "));
        Serial.print(t_build_J / 1000.0, 3);
        Serial.println(F(" ms"));
        
        float t_hamiltonian = (t_build_G + t_build_H + t_build_J) / 1000.0;
        Serial.print(F("Subtotal Hamiltonianas: "));
        Serial.print(t_hamiltonian, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 3: DECOMPOSIÇÃO QZ (CRÍTICA) ---"));
        Serial.print(F("5. Eigen::GeneralizedEigenSolver: "));
        Serial.print(t_qz_decomp / 1000.0, 3);
        Serial.print(F(" ms ("));
        Serial.print((t_qz_decomp / (float)t_total) * 100, 1);
        Serial.println(F("%)"));
        
        Serial.println(F("\n--- FASE 4: ORDENAÇÃO E EXTRAÇÃO ---"));
        Serial.print(F("6. Ordenar autovalores estáveis (|λ| < 1): "));
        Serial.print(t_sort_eig / 1000.0, 3);
        Serial.println(F(" ms"));
        Serial.print(F("   Autovalores estáveis encontrados: "));
        Serial.print(stable_indices.size());
        Serial.print(F(" / "));
        Serial.println(n);
        
        Serial.print(F("7. Extrair subespaço invariante (U11, U21): "));
        Serial.print(t_extract_subspace / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 5: SOLUÇÃO DA DARE ---"));
        Serial.print(F("8. Calcular P = U21 * inv(U11): "));
        Serial.print(t_calc_P / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 6: GANHO LQR ---"));
        Serial.print(F("9. Calcular K = inv(R+B^T*P*B)*B^T*P*A: "));
        Serial.print(t_calc_K / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\n--- FASE 7: VERIFICAÇÃO ---"));
        Serial.print(F("10. Verificar estabilidade (malha fechada): "));
        Serial.print(t_verify / 1000.0, 3);
        Serial.println(F(" ms"));
        Serial.print(F("    Sistema em malha fechada: "));
        Serial.println(stable ? F("ESTÁVEL") : F("INSTÁVEL"));
        
        Serial.println(F("\n--- RESUMO GERAL ---"));
        Serial.print(F("TEMPO TOTAL: "));
        Serial.print(t_total / 1000.0, 3);
        Serial.println(F(" ms"));
        
        Serial.println(F("\nDistribuição por fase:"));
        float total_ms = t_total / 1000.0;
        
        Serial.print(F("  Preparação:          "));
        Serial.print((t_alloc_start / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.print(F("  Hamiltonianas:       "));
        Serial.print((t_hamiltonian / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.print(F("  Decomposição QZ:     "));
        Serial.print((t_qz_decomp / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("% (GARGALO)"));
        
        Serial.print(F("  Ordenação/Extração:  "));
        Serial.print(((t_sort_eig + t_extract_subspace) / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.print(F("  Solução P:           "));
        Serial.print((t_calc_P / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.print(F("  Ganho K:             "));
        Serial.print((t_calc_K / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.print(F("  Verificação:         "));
        Serial.print((t_verify / 1000.0 / total_ms) * 100, 1);
        Serial.println(F("%"));
        
        Serial.println(F("\n--- COMPARAÇÃO COM MÉTODO ITERATIVO ---"));
        Serial.println(F("Vantagens do Schur:"));
        Serial.println(F("  + Convergência garantida (sem iterações)"));
        Serial.println(F("  + Solução única e numericamente estável"));
        Serial.println(F("  + Não depende de chute inicial"));
        Serial.println(F("Desvantagens:"));
        Serial.println(F("  - Maior custo computacional para sistemas pequenos"));
        Serial.println(F("  - Requer decomposição QZ (operação densa)"));
        
        Serial.println(F("======================================================================\n"));
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
