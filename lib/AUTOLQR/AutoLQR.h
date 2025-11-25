#ifndef AUTO_LQR_H
#define AUTO_LQR_H

#include <Arduino.h>
#include "MatrixOperations.h"

class AutoLQR : public MatrixOperations {
public:
    /**
     * @brief Construct a new AutoLQR controller
     * @param stateSize Number of state variables
     * @param controlSize Number of control inputs
     */
    AutoLQR(int stateSize, int controlSize);

    /**
     * @brief Destroy the AutoLQR controller and free memory
     */
    ~AutoLQR();

    /**
     * @brief Set the system state matrix A
     * @param A Pointer to state matrix (stateSize x stateSize)
     * @return true if successful, false otherwise
     */
    bool setStateMatrix(const float* A);

    /**
     * @brief Set the input matrix B
     * @param B Pointer to input matrix (stateSize x controlSize)
     * @return true if successful, false otherwise
     */
    bool setInputMatrix(const float* B);

    /**
     * @brief Set the cost matrices Q and R
     * @param Q Pointer to state cost matrix (stateSize x stateSize)
     * @param R Pointer to control cost matrix (controlSize x controlSize)
     * @return true if successful, false otherwise
     */
    bool setCostMatrices(const float* Q, const float* R);

    /**
     * @brief Compute optimal feedback gains
     * @return true if successful, false if computation fails
     */
    bool computeGains();

    /**
     * @brief Update the controller with current state
     * @param currentState Pointer to current state vector (stateSize)
     */
    void updateState(const float* currentState);

    /**
     * @brief Calculate control inputs based on current state
     * @param controlOutput Pointer to control output vector (controlSize)
     */
    void calculateControl(float* controlOutput);

    /**
     * @brief Set pre-computed gain values
     * @param K Pointer to gain matrix (controlSize x stateSize)
     */
    void setGains(const float* K);

    /**
     * @brief Check if the system is controllable
     * @return true if controllable, false otherwise
     */
    bool isSystemControllable();

    /**
     * @brief Get the solution of the Riccati equation
     * @return Pointer to the P matrix (stateSize x stateSize)
     */
    const float* getRicattiSolution() const;

    /**
     * @brief Estimate feedforward gain for steady-state tracking
     * @param ffGain Pointer to feedforward gain vector (controlSize)
     * @param desiredState Pointer to desired state vector (stateSize)
     */
    void estimateFeedforwardGain(float* ffGain, const float* desiredState);

    /**
     * @brief Estimate time to convergence
     * @param convergenceThreshold Threshold for considering system converged (default: 0.05)
     * @return Estimated time in seconds, or -1 if estimation fails
     */
    float estimateConvergenceTime(float convergenceThreshold = 0.05f);

    /**
     * @brief Export computed gains to external array
     * @param exportedK Pointer to destination array (controlSize x stateSize)
     * @return true if successful, false otherwise
     */
    bool exportGains(float* exportedK);

    /**
     * @brief Calculate expected cost from current state
     * @return Expected cost value, or -1 if calculation fails
     */
    float calculateExpectedCost();

    /**
     * @brief Compute Kr gains
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixKr();

    /**
     * @brief Export Kr gains to external array
     * @param exportedKr Pointer to destination array
     * @return true if successful, false otherwise
     */
    bool exportKr(float* exportedKr);

    /**
     * @brief Update the reference values
     * @param newReference Pointer to new reference vector
     */
    void updateReference(const float* newReference);

private:
    int stateSize; ///< Number of state variables
    int controlSize; ///< Number of control inputs

    float* A; ///< State matrix
    float* B; ///< Input matrix
    float* Q; ///< State cost matrix
    float* R; ///< Control cost matrix
    float* K; ///< Control gain matrix
    float* state; ///< Current state
    float* P; ///< Riccati equation solution
    float* Kr; ///< Kr gain matrix
    float* reference; ///< To store reference values

    /**
     * @brief Compute the optimal gain matrix by solving DARE (iterative method)
     * @return true if successful, false otherwise
     */
    bool computeGainMatrix();
    
    /**
     * @brief Compute the optimal gain matrix using Schur method (QZ decomposition)
     * Solves DARE using generalized Schur decomposition for direct solution
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixSchur();

    /**
     * @brief Compute the optimal gain matrix using Van Dooren's method
     * Solves DARE using Van Dooren's algorithm for numerical stability
     * @return true if successful, false otherwise
     */    
    bool computeGainMatrixVanDooren();

    /**
     * @brief Compute the optimal gain matrix using SDA method
     * Solves DARE using State-dependent Riccati Equation for adaptive control
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixSDA();
};

#endif
