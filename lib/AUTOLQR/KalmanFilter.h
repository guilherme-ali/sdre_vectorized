#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "MatrixOperations.h"

/**
 * @class KalmanFilter
 * @brief Implementa um filtro de Kalman linear usando uma biblioteca de operações de matriz otimizada.
 *
 * Esta classe encapsula a lógica de um filtro de Kalman, incluindo as etapas de
 * predição e atualização. Ela é inicializada com as matrizes do modelo do sistema (A, B, C)
 * e as matrizes de covariância de ruído (Q, R). A classe gerencia internamente a alocação
 * de memória para o estado, covariância e matrizes temporárias necessárias para os cálculos.
 */
class KalmanFilter {
public:
    /**
     * @brief Construtor da classe KalmanFilter.
     * @param state_dim Dimensão do vetor de estado (n).
     * @param control_dim Dimensão do vetor de controle (p).
     * @param measurement_dim Dimensão do vetor de medição (m).
     */
    KalmanFilter(int state_dim, int control_dim, int measurement_dim);

    /**
     * @brief Destrutor. Libera toda a memória alocada dinamicamente.
     */
    ~KalmanFilter();

    /**
     * @brief Inicializa o filtro com as matrizes do sistema e o estado inicial.
     * @param A Matriz de transição de estado (n x n).
     * @param B Matriz de entrada de controle (n x p).
     * @param C Matriz de medição (m x n).
     * @param Q Covariância do ruído do processo (n x n).
     * @param R Covariância do ruído da medição (m x m).
     * @param x0 Vetor de estado inicial (n x 1).
     * @param P0 Matriz de covariância da estimativa inicial (n x n).
     */
    void init(const float* A, const float* B, const float* C,
              const float* Q, const float* R, const float* x0, const float* P0);

    /**
     * @brief Executa a etapa de predição do filtro de Kalman.
     * @param u Vetor de entrada de controle (p x 1). Pode ser nullptr se não houver entrada.
     */
    void predict(const float* u);

    /**
     * @brief Executa a etapa de atualização do filtro de Kalman com uma nova medição.
     * @param z Vetor de medição (m x 1).
     */
    void update(const float* z);

    /**
     * @brief Retorna o vetor de estado estimado atual.
     * @return Ponteiro constante para o vetor de estado (x_est).
     */
    const float* getState() const { return x_est; }

    /**
     * @brief Retorna a matriz de covariância da estimativa atual.
     * @return Ponteiro constante para a matriz de covariância (P_est).
     */
    const float* getCovariance() const { return P_est; }

private:
    // Dimensões
    int n; // Dimensão do estado
    int p; // Dimensão do controle
    int m; // Dimensão da medição

    // Matrizes do Modelo (constantes após inicialização)
    float* A;   // n x n
    float* B;   // n x p
    float* C;   // m x n
    float* Q;   // n x n
    float* R;   // m x m
    float* A_T; // n x n (Transposta de A)
    float* C_T; // n x m (Transposta de C)

    // Estado do Filtro
    float* x_est; // n x 1 (Vetor de estado estimado)
    float* P_est; // n x n (Matriz de covariância da estimativa)

    // Variáveis da Etapa de Predição
    float* x_pred; // n x 1
    float* P_pred; // n x n

    // Variáveis da Etapa de Atualização
    float* K; // n x m (Ganho de Kalman)
    float* y; // m x 1 (Inovação)

    // Matrizes temporárias para evitar alocação repetida
    float* temp_n1_1;
    float* temp_n1_2;
    float* temp_m1_1;
    float* temp_nn_1;
    float* temp_nn_2;
    float* temp_mn_1;
    float* temp_nm_1;
    float* temp_mm_1;
    float* temp_mm_2;
    float* S_inv;
};

#endif // KALMAN_FILTER_H