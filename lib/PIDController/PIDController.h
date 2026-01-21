#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Classe para controle PID de atitude de drone
 * 
 * Implementa controladores PID independentes para roll, pitch e yaw,
 * compatível com a interface do SDRE/AutoLQR.
 */
class PIDController {
public:
    /**
     * @brief Construtor do PIDController
     * @param stateSize Número de variáveis de estado (deve ser 6 para compatibilidade)
     * @param controlSize Número de entradas de controle (deve ser 3 para compatibilidade)
     */
    PIDController(int stateSize, int controlSize);

    /**
     * @brief Destrutor do PIDController - libera memória
     */
    ~PIDController();

    /**
     * @brief Define os ganhos PID para o eixo roll
     * @param kp Ganho proporcional
     * @param ki Ganho integral
     * @param kd Ganho derivativo
     */
    void setRollGains(float kp, float ki, float kd);

    /**
     * @brief Define os ganhos PID para o eixo pitch
     * @param kp Ganho proporcional
     * @param ki Ganho integral
     * @param kd Ganho derivativo
     */
    void setPitchGains(float kp, float ki, float kd);

    /**
     * @brief Define os ganhos PID para o eixo yaw
     * @param kp Ganho proporcional
     * @param ki Ganho integral
     * @param kd Ganho derivativo
     */
    void setYawGains(float kp, float ki, float kd);

    /**
     * @brief Define todos os ganhos PID de uma vez
     * @param roll_kp Ganho proporcional do roll
     * @param roll_ki Ganho integral do roll
     * @param roll_kd Ganho derivativo do roll
     * @param pitch_kp Ganho proporcional do pitch
     * @param pitch_ki Ganho integral do pitch
     * @param pitch_kd Ganho derivativo do pitch
     * @param yaw_kp Ganho proporcional do yaw
     * @param yaw_ki Ganho integral do yaw
     * @param yaw_kd Ganho derivativo do yaw
     */
    void setAllGains(float roll_kp, float roll_ki, float roll_kd,
                     float pitch_kp, float pitch_ki, float pitch_kd,
                     float yaw_kp, float yaw_ki, float yaw_kd);

    /**
     * @brief Define os limites de anti-windup do termo integral
     * @param roll_limit Limite para o integral do roll
     * @param pitch_limit Limite para o integral do pitch
     * @param yaw_limit Limite para o integral do yaw
     */
    void setIntegralLimits(float roll_limit, float pitch_limit, float yaw_limit);

    /**
     * @brief Define os limites de saída dos sinais de controle
     * @param min_output Saída mínima de controle
     * @param max_output Saída máxima de controle
     */
    void setOutputLimits(float min_output, float max_output);

    /**
     * @brief Atualiza o controlador com o estado atual
     * Formato do vetor de estados: [roll, pitch, yaw, p, q, r]
     * @param currentState Ponteiro para o vetor de estado atual (stateSize)
     */
    void updateState(const float* currentState);

    /**
     * @brief Atualiza os valores de referência
     * Formato da referência: [phi_desejado, theta_desejado, psi_desejado]
     * @param newReference Ponteiro para o vetor de referência (controlSize)
     */
    void updateReference(const float* newReference);

    /**
     * @brief Calcula as entradas de controle baseado no estado atual e referência
     * Formato de saída: [tau_roll, tau_pitch, tau_yaw] (torques)
     * @param controlOutput Ponteiro para o vetor de saída de controle (controlSize)
     */
    void calculateControl(float* controlOutput);

    /**
     * @brief Reseta todos os termos integrais para zero
     */
    void resetIntegrals();

    /**
     * @brief Define o tempo de amostragem para cálculos integral e derivativo
     * @param dt Tempo de amostragem em segundos
     */
    void setSamplingTime(float dt);

    /**
     * @brief Habilita ou desabilita o filtro derivativo
     * @param enable True para habilitar filtro passa-baixa no termo derivativo
     * @param alpha Coeficiente do filtro (0-1), menor = mais filtragem
     */
    void setDerivativeFilter(bool enable, float alpha = 0.1f);

    /**
     * @brief Obtém os valores integrais atuais para debug
     * @param roll_int Saída do integral de roll
     * @param pitch_int Saída do integral de pitch
     * @param yaw_int Saída do integral de yaw
     */
    void getIntegrals(float& roll_int, float& pitch_int, float& yaw_int) const;

    /**
     * @brief Obtém os valores de erro atuais para debug
     * @param roll_err Saída do erro de roll
     * @param pitch_err Saída do erro de pitch
     * @param yaw_err Saída do erro de yaw
     */
    void getErrors(float& roll_err, float& pitch_err, float& yaw_err) const;

    /**
     * @brief Obtém os ganhos PID para cada eixo
     * @param axis 0=Roll, 1=Pitch, 2=Yaw
     * @param kp Saída do ganho proporcional
     * @param ki Saída do ganho integral
     * @param kd Saída do ganho derivativo
     */
    void getGains(int axis, float& kp, float& ki, float& kd) const;

    // ===== Métodos de compatibilidade com AutoLQR =====
    
    /**
     * @brief Placeholder para compatibilidade com interface AutoLQR
     * PID não usa matrizes de sistema, mas é necessário para compatibilidade
     * @param A Ponteiro para matriz de estados (ignorado)
     * @return Sempre retorna true
     */
    bool setStateMatrix(const float* A);

    /**
     * @brief Placeholder para compatibilidade com interface AutoLQR
     * @param B Ponteiro para matriz de entrada (ignorado)
     * @return Sempre retorna true
     */
    bool setInputMatrix(const float* B);

    /**
     * @brief Placeholder para compatibilidade com interface AutoLQR
     * @param Q Ponteiro para matriz de custo de estados (ignorado)
     * @param R Ponteiro para matriz de custo de controle (ignorado)
     * @return Sempre retorna true
     */
    bool setCostMatrices(const float* Q, const float* R);

    /**
     * @brief Placeholder para compatibilidade com interface AutoLQR
     * PID não computa ganhos dinamicamente como SDRE
     * @param method Nome do método (ignorado)
     * @return Sempre retorna true
     */
    bool computeGains(const char* method = "PID");

    /**
     * @brief Obtém solução de Riccati (para compatibilidade)
     * Retorna nullptr pois PID não tem este conceito
     * @return nullptr
     */
    const float* getRicattiSolution() const;

    /**
     * @brief Exporta ganhos atuais (para debug/exibição)
     * @param exportedK Ponteiro para array de destino (controlSize x stateSize)
     * @return true se bem-sucedido
     */
    bool exportGains(float* exportedK);

private:
    int stateSize;      ///< Número de variáveis de estado
    int controlSize;    ///< Número de entradas de controle

    // Ganhos PID para cada eixo
    float roll_kp, roll_ki, roll_kd;
    float pitch_kp, pitch_ki, pitch_kd;
    float yaw_kp, yaw_ki, yaw_kd;

    // Termos integrais
    float roll_integral;
    float pitch_integral;
    float yaw_integral;

    // Erros anteriores para cálculo derivativo
    float prev_roll_error;
    float prev_pitch_error;
    float prev_yaw_error;

    // Derivadas filtradas
    float filtered_roll_derivative;
    float filtered_pitch_derivative;
    float filtered_yaw_derivative;

    // Limites integrais (anti-windup)
    float roll_int_limit;
    float pitch_int_limit;
    float yaw_int_limit;

    // Limites de saída
    float output_min;
    float output_max;

    // Armazenamento de estado e referência
    float* state;
    float* reference;

    // Temporização
    float dt;               ///< Tempo de amostragem
    bool first_run;         ///< Flag para primeira iteração

    // Filtro derivativo
    bool use_derivative_filter;
    float derivative_alpha;

    // Erros atuais (para debug)
    float current_roll_error;
    float current_pitch_error;
    float current_yaw_error;

    /**
     * @brief Limita um valor dentro de limites
     */
    float clampValue(float value, float min_val, float max_val);

    /**
     * @brief Computa PID para um único eixo
     */
    float computeAxisPID(float error, float rate, float& integral, 
                         float& prev_error, float& filtered_derivative,
                         float kp, float ki, float kd, float int_limit);
};

#endif // PID_CONTROLLER_H
