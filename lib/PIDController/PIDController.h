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
     * @brief Construct a new PIDController
     * @param stateSize Number of state variables (deve ser 6 para compatibilidade)
     * @param controlSize Number of control inputs (deve ser 3 para compatibilidade)
     */
    PIDController(int stateSize, int controlSize);

    /**
     * @brief Destroy the PIDController and free memory
     */
    ~PIDController();

    /**
     * @brief Set PID gains for roll axis
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setRollGains(float kp, float ki, float kd);

    /**
     * @brief Set PID gains for pitch axis
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPitchGains(float kp, float ki, float kd);

    /**
     * @brief Set PID gains for yaw axis
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setYawGains(float kp, float ki, float kd);

    /**
     * @brief Set all PID gains at once
     * @param roll_kp Roll proportional gain
     * @param roll_ki Roll integral gain
     * @param roll_kd Roll derivative gain
     * @param pitch_kp Pitch proportional gain
     * @param pitch_ki Pitch integral gain
     * @param pitch_kd Pitch derivative gain
     * @param yaw_kp Yaw proportional gain
     * @param yaw_ki Yaw integral gain
     * @param yaw_kd Yaw derivative gain
     */
    void setAllGains(float roll_kp, float roll_ki, float roll_kd,
                     float pitch_kp, float pitch_ki, float pitch_kd,
                     float yaw_kp, float yaw_ki, float yaw_kd);

    /**
     * @brief Set integral windup limits
     * @param roll_limit Limit for roll integral term
     * @param pitch_limit Limit for pitch integral term
     * @param yaw_limit Limit for yaw integral term
     */
    void setIntegralLimits(float roll_limit, float pitch_limit, float yaw_limit);

    /**
     * @brief Set output limits for control signals
     * @param min_output Minimum control output
     * @param max_output Maximum control output
     */
    void setOutputLimits(float min_output, float max_output);

    /**
     * @brief Update the controller with current state
     * State vector format: [roll, pitch, yaw, p, q, r]
     * @param currentState Pointer to current state vector (stateSize)
     */
    void updateState(const float* currentState);

    /**
     * @brief Update the reference values
     * Reference format: [phi_desired, theta_desired, psi_desired]
     * @param newReference Pointer to new reference vector (controlSize)
     */
    void updateReference(const float* newReference);

    /**
     * @brief Calculate control inputs based on current state and reference
     * Output format: [tau_roll, tau_pitch, tau_yaw] (torques)
     * @param controlOutput Pointer to control output vector (controlSize)
     */
    void calculateControl(float* controlOutput);

    /**
     * @brief Reset all integral terms to zero
     */
    void resetIntegrals();

    /**
     * @brief Set the sampling time for integral and derivative calculations
     * @param dt Sampling time in seconds
     */
    void setSamplingTime(float dt);

    /**
     * @brief Enable or disable derivative filtering
     * @param enable True to enable low-pass filter on derivative term
     * @param alpha Filter coefficient (0-1), lower = more filtering
     */
    void setDerivativeFilter(bool enable, float alpha = 0.1f);

    /**
     * @brief Get the current integral values for debugging
     * @param roll_int Output roll integral
     * @param pitch_int Output pitch integral
     * @param yaw_int Output yaw integral
     */
    void getIntegrals(float& roll_int, float& pitch_int, float& yaw_int) const;

    /**
     * @brief Get the current error values for debugging
     * @param roll_err Output roll error
     * @param pitch_err Output pitch error
     * @param yaw_err Output yaw error
     */
    void getErrors(float& roll_err, float& pitch_err, float& yaw_err) const;

    /**
     * @brief Get the PID gains for each axis
     * @param axis 0=Roll, 1=Pitch, 2=Yaw
     * @param kp Output proportional gain
     * @param ki Output integral gain
     * @param kd Output derivative gain
     */
    void getGains(int axis, float& kp, float& ki, float& kd) const;

    // ===== Métodos de compatibilidade com AutoLQR =====
    
    /**
     * @brief Placeholder for compatibility with AutoLQR interface
     * PID doesn't use system matrices, but this is needed for interface compatibility
     * @param A Pointer to state matrix (ignored)
     * @return Always returns true
     */
    bool setStateMatrix(const float* A);

    /**
     * @brief Placeholder for compatibility with AutoLQR interface
     * @param B Pointer to input matrix (ignored)
     * @return Always returns true
     */
    bool setInputMatrix(const float* B);

    /**
     * @brief Placeholder for compatibility with AutoLQR interface
     * @param Q Pointer to state cost matrix (ignored)
     * @param R Pointer to control cost matrix (ignored)
     * @return Always returns true
     */
    bool setCostMatrices(const float* Q, const float* R);

    /**
     * @brief Placeholder for compatibility with AutoLQR interface
     * PID doesn't compute gains dynamically like SDRE
     * @param method Method name (ignored)
     * @return Always returns true
     */
    bool computeGains(const char* method = "PID");

    /**
     * @brief Get P gains (equivalent to Riccati solution for compatibility)
     * Returns nullptr since PID doesn't have this concept
     * @return nullptr
     */
    const float* getRicattiSolution() const;

    /**
     * @brief Export current gains (for debugging/display)
     * @param exportedK Pointer to destination array (controlSize x stateSize)
     * @return true if successful
     */
    bool exportGains(float* exportedK);

private:
    int stateSize;      ///< Number of state variables
    int controlSize;    ///< Number of control inputs

    // PID gains for each axis
    float roll_kp, roll_ki, roll_kd;
    float pitch_kp, pitch_ki, pitch_kd;
    float yaw_kp, yaw_ki, yaw_kd;

    // Integral terms
    float roll_integral;
    float pitch_integral;
    float yaw_integral;

    // Previous errors for derivative calculation
    float prev_roll_error;
    float prev_pitch_error;
    float prev_yaw_error;

    // Filtered derivatives
    float filtered_roll_derivative;
    float filtered_pitch_derivative;
    float filtered_yaw_derivative;

    // Integral limits (anti-windup)
    float roll_int_limit;
    float pitch_int_limit;
    float yaw_int_limit;

    // Output limits
    float output_min;
    float output_max;

    // State and reference storage
    float* state;
    float* reference;

    // Timing
    float dt;               ///< Sampling time
    bool first_run;         ///< Flag for first iteration

    // Derivative filter
    bool use_derivative_filter;
    float derivative_alpha;

    // Current errors (for debugging)
    float current_roll_error;
    float current_pitch_error;
    float current_yaw_error;

    /**
     * @brief Clamp a value within limits
     */
    float clampValue(float value, float min_val, float max_val);

    /**
     * @brief Compute single axis PID
     */
    float computeAxisPID(float error, float rate, float& integral, 
                         float& prev_error, float& filtered_derivative,
                         float kp, float ki, float kd, float int_limit);
};

#endif // PID_CONTROLLER_H
