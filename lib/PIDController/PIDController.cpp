#include "PIDController.h"
#include <math.h>

PIDController::PIDController(int stateSize, int controlSize)
    : stateSize(stateSize)
    , controlSize(controlSize)
    // Ganhos PID padrão (valores conservadores para um drone pequeno)
    , roll_kp(2.0f), roll_ki(0.5f), roll_kd(0.1f)
    , pitch_kp(2.0f), pitch_ki(0.5f), pitch_kd(0.1f)
    , yaw_kp(1.5f), yaw_ki(0.2f), yaw_kd(0.05f)
    // Inicializa integrais
    , roll_integral(0.0f)
    , pitch_integral(0.0f)
    , yaw_integral(0.0f)
    // Inicializa erros anteriores
    , prev_roll_error(0.0f)
    , prev_pitch_error(0.0f)
    , prev_yaw_error(0.0f)
    // Inicializa derivadas filtradas
    , filtered_roll_derivative(0.0f)
    , filtered_pitch_derivative(0.0f)
    , filtered_yaw_derivative(0.0f)
    // Limites integrais padrão
    , roll_int_limit(1.0f)
    , pitch_int_limit(1.0f)
    , yaw_int_limit(0.5f)
    // Limites de saída padrão
    , output_min(-10.0f)
    , output_max(10.0f)
    // Estado e referência
    , state(nullptr)
    , reference(nullptr)
    // Temporização
    , dt(0.012f)  // Padrão 12ms como no loop principal
    , first_run(true)
    // Filtro derivativo
    , use_derivative_filter(true)
    , derivative_alpha(0.2f)
    // Erros atuais
    , current_roll_error(0.0f)
    , current_pitch_error(0.0f)
    , current_yaw_error(0.0f)
{
    if (stateSize > 0 && controlSize > 0) {
        state = new float[stateSize]();
        reference = new float[controlSize]();
    }
}

PIDController::~PIDController() {
    delete[] state;
    delete[] reference;
}

void PIDController::setRollGains(float kp, float ki, float kd) {
    roll_kp = kp;
    roll_ki = ki;
    roll_kd = kd;
}

void PIDController::setPitchGains(float kp, float ki, float kd) {
    pitch_kp = kp;
    pitch_ki = ki;
    pitch_kd = kd;
}

void PIDController::setYawGains(float kp, float ki, float kd) {
    yaw_kp = kp;
    yaw_ki = ki;
    yaw_kd = kd;
}

void PIDController::setAllGains(float r_kp, float r_ki, float r_kd,
                                 float p_kp, float p_ki, float p_kd,
                                 float y_kp, float y_ki, float y_kd) {
    setRollGains(r_kp, r_ki, r_kd);
    setPitchGains(p_kp, p_ki, p_kd);
    setYawGains(y_kp, y_ki, y_kd);
}

void PIDController::setIntegralLimits(float roll_limit, float pitch_limit, float yaw_limit) {
    roll_int_limit = roll_limit;
    pitch_int_limit = pitch_limit;
    yaw_int_limit = yaw_limit;
}

void PIDController::setOutputLimits(float min_output, float max_output) {
    output_min = min_output;
    output_max = max_output;
}

void PIDController::updateState(const float* currentState) {
    if (!currentState || !state) return;
    
    for (int i = 0; i < stateSize; i++) {
        state[i] = currentState[i];
    }
}

void PIDController::updateReference(const float* newReference) {
    if (!newReference || !reference) return;
    
    for (int i = 0; i < controlSize; i++) {
        reference[i] = newReference[i];
    }
}

float PIDController::clampValue(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float PIDController::computeAxisPID(float error, float rate, float& integral,
                                     float& prev_error, float& filtered_derivative,
                                     float kp, float ki, float kd, float int_limit) {
    // Termo proporcional
    float P = kp * error;
    
    // Termo integral com anti-windup
    integral += error * dt;
    integral = clampValue(integral, -int_limit, int_limit);
    float I = ki * integral;
    
    // Termo derivativo
    // Opção 1: Usar taxa angular diretamente (mais estável)
    // Opção 2: Usar derivada do erro (tradicional)
    float D;
    
    if (first_run) {
        D = 0.0f;
        filtered_derivative = 0.0f;
    } else {
        // Usa taxa angular diretamente para melhor imunidade a ruído
        // rate já contém p, q ou r do giroscópio
        float raw_derivative = -rate; // Negativo porque queremos opor à taxa
        
        // Aplica filtro passa-baixa na derivada se habilitado
        if (use_derivative_filter) {
            filtered_derivative = derivative_alpha * raw_derivative + 
                                  (1.0f - derivative_alpha) * filtered_derivative;
            D = kd * filtered_derivative;
        } else {
            D = kd * raw_derivative;
        }
    }
    
    prev_error = error;
    
    // Soma todos os termos
    float output = P + I + D;
    
    // Limita a saída
    output = clampValue(output, output_min, output_max);
    
    return output;
}

void PIDController::calculateControl(float* controlOutput) {
    if (!controlOutput || !state || !reference) return;
    
    // Vetor de estados: [roll, pitch, yaw, p, q, r]
    float roll = state[0];
    float pitch = state[1];
    float yaw = state[2];
    float p = state[3];  // Taxa de roll
    float q = state[4];  // Taxa de pitch
    float r = state[5];  // Taxa de yaw
    
    // Referência: [phi_desejado, theta_desejado, psi_desejado]
    float phi_desired = reference[0];
    float theta_desired = reference[1];
    float psi_desired = reference[2];
    
    // Calcula erros
    current_roll_error = phi_desired - roll;
    current_pitch_error = theta_desired - pitch;
    current_yaw_error = psi_desired - yaw;
    
    // Normaliza erro de yaw para [-pi, pi]
    while (current_yaw_error > M_PI) current_yaw_error -= 2.0f * M_PI;
    while (current_yaw_error < -M_PI) current_yaw_error += 2.0f * M_PI;
    
    // Computa PID para cada eixo
    // Saída é torque: [tau_roll, tau_pitch, tau_yaw]
    controlOutput[0] = computeAxisPID(current_roll_error, p, roll_integral,
                                       prev_roll_error, filtered_roll_derivative,
                                       roll_kp, roll_ki, roll_kd, roll_int_limit);
    
    controlOutput[1] = computeAxisPID(current_pitch_error, q, pitch_integral,
                                       prev_pitch_error, filtered_pitch_derivative,
                                       pitch_kp, pitch_ki, pitch_kd, pitch_int_limit);
    
    controlOutput[2] = computeAxisPID(current_yaw_error, r, yaw_integral,
                                       prev_yaw_error, filtered_yaw_derivative,
                                       yaw_kp, yaw_ki, yaw_kd, yaw_int_limit);
    
    first_run = false;
}

void PIDController::resetIntegrals() {
    roll_integral = 0.0f;
    pitch_integral = 0.0f;
    yaw_integral = 0.0f;
    first_run = true;
    filtered_roll_derivative = 0.0f;
    filtered_pitch_derivative = 0.0f;
    filtered_yaw_derivative = 0.0f;
}

void PIDController::setSamplingTime(float sampling_time) {
    dt = sampling_time;
}

void PIDController::setDerivativeFilter(bool enable, float alpha) {
    use_derivative_filter = enable;
    derivative_alpha = clampValue(alpha, 0.01f, 1.0f);
}

void PIDController::getIntegrals(float& roll_int, float& pitch_int, float& yaw_int) const {
    roll_int = roll_integral;
    pitch_int = pitch_integral;
    yaw_int = yaw_integral;
}

void PIDController::getErrors(float& roll_err, float& pitch_err, float& yaw_err) const {
    roll_err = current_roll_error;
    pitch_err = current_pitch_error;
    yaw_err = current_yaw_error;
}

void PIDController::getGains(int axis, float& kp, float& ki, float& kd) const {
    switch(axis) {
        case 0: // Roll
            kp = roll_kp;
            ki = roll_ki;
            kd = roll_kd;
            break;
        case 1: // Pitch
            kp = pitch_kp;
            ki = pitch_ki;
            kd = pitch_kd;
            break;
        case 2: // Yaw
            kp = yaw_kp;
            ki = yaw_ki;
            kd = yaw_kd;
            break;
        default:
            kp = ki = kd = 0.0f;
            break;
    }
}

// ===== Métodos de compatibilidade com AutoLQR =====

bool PIDController::setStateMatrix(const float* A) {
    // PID não usa matriz de estados, mas retorna true para compatibilidade
    (void)A; // Suprime aviso de parâmetro não usado
    return true;
}

bool PIDController::setInputMatrix(const float* B) {
    // PID não usa matriz de entrada, mas retorna true para compatibilidade
    (void)B;
    return true;
}

bool PIDController::setCostMatrices(const float* Q, const float* R) {
    // PID não usa matrizes de custo, mas retorna true para compatibilidade
    (void)Q;
    (void)R;
    return true;
}

bool PIDController::computeGains(const char* method) {
    // PID não computa ganhos dinamicamente como SDRE
    // Os ganhos são definidos manualmente via setXXXGains()
    (void)method;
    return true;
}

const float* PIDController::getRicattiSolution() const {
    // PID não tem solução de Riccati
    return nullptr;
}

bool PIDController::exportGains(float* exportedK) {
    if (!exportedK) return false;
    
    // Exporta os ganhos PID em formato similar à matriz K do LQR
    // Formato: [controlSize x stateSize] = [3 x 6]
    // Cada linha representa os ganhos para um eixo
    // Coluna 0-2: ganhos proporcionais (para roll, pitch, yaw)
    // Coluna 3-5: ganhos derivativos (para p, q, r)
    
    // Inicializa com zeros
    for (int i = 0; i < controlSize * stateSize; i++) {
        exportedK[i] = 0.0f;
    }
    
    // Ganhos do controlador de Roll
    exportedK[0 * stateSize + 0] = roll_kp;  // K[0,0] = roll_kp (ganho para erro de roll)
    exportedK[0 * stateSize + 3] = roll_kd;  // K[0,3] = roll_kd (ganho para taxa p)
    
    // Ganhos do controlador de Pitch
    exportedK[1 * stateSize + 1] = pitch_kp; // K[1,1] = pitch_kp (ganho para erro de pitch)
    exportedK[1 * stateSize + 4] = pitch_kd; // K[1,4] = pitch_kd (ganho para taxa q)
    
    // Ganhos do controlador de Yaw
    exportedK[2 * stateSize + 2] = yaw_kp;   // K[2,2] = yaw_kp (ganho para erro de yaw)
    exportedK[2 * stateSize + 5] = yaw_kd;   // K[2,5] = yaw_kd (ganho para taxa r)
    
    return true;
}
