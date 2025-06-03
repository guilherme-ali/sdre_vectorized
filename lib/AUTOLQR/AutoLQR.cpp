#include "AutoLQR.h"
#include <math.h>

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
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Check if system is controllable
    if (!isSystemControllable()) {
        return false;
    }

    // Iterative DARE solver for small systems
    float* P_next = new float[stateSize * stateSize]();
    float* BT = new float[controlSize * stateSize]();
    float* AT = new float[stateSize * stateSize]();
    float* temp1 = new float[controlSize * stateSize]();
    float* temp2 = new float[controlSize * controlSize]();
    float* temp3 = new float[controlSize * stateSize]();
    float* temp4 = new float[stateSize * stateSize]();
    float* temp5 = new float[stateSize * stateSize]();

    // Check if P has been initialized with non-zero values
    bool isPInitialized = false;
    for (int i = 0; i < stateSize * stateSize; i++) {
        if (fabs(P[i]) > 1e-6) {
            isPInitialized = true;
            break;
        }
    }

    // Initialize P as Q only if P hasn't been calculated before
    if (!isPInitialized) {
        matrixCopy(Q, P, stateSize * stateSize);
    }
    // Otherwise, keep using the previously calculated P

    // Compute B transpose and A transpose
    transposeMatrix(B, BT, stateSize, controlSize);
    transposeMatrix(A, AT, stateSize, stateSize);

    // Iterate to solve DARE: P = A'PA - A'PB(R + B'PB)^(-1)B'PA + Q
    const int maxIterations = 50;
    const float tolerance = 1e-3;

    for (int iter = 0; iter < maxIterations; iter++) {
        // temp1 = B'P
        matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);

        // temp2 = B'PB
        matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

        // temp2 = R + B'PB
        matrixAdd(R, temp2, temp2, controlSize, controlSize);

        // Invert (R + B'PB)
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

        // temp3 = (R + B'PB)^(-1) * B'P
        matrixMultiply(temp2, temp1, temp3, controlSize, controlSize, stateSize);

        // temp4 = A'PA
        matrixMultiply(AT, P, temp4, stateSize, stateSize, stateSize);
        matrixMultiply(temp4, A, P_next, stateSize, stateSize, stateSize);

        // temp5 = A'PB
        matrixMultiply(AT, P, temp4, stateSize, stateSize, stateSize);
        matrixMultiply(temp4, B, temp5, stateSize, stateSize, controlSize);

        // temp4 = A'PB * (R + B'PB)^(-1) * B'PA
        matrixMultiply(temp5, temp3, temp4, stateSize, controlSize, stateSize);
        matrixMultiply(temp4, A, temp5, stateSize, stateSize, stateSize);

        // P_next = A'PA - A'PB * (R + B'PB)^(-1) * B'PA + Q
        matrixSubtract(P_next, temp5, P_next, stateSize, stateSize);
        matrixAdd(P_next, Q, P_next, stateSize, stateSize);

        // Check convergence
        float diff = 0;
        for (int i = 0; i < stateSize * stateSize; i++) {
            diff += fabs(P_next[i] - P[i]);
        }

        // Update P for next iteration
        matrixCopy(P_next, P, stateSize * stateSize);

        if (diff < tolerance) {
            break; // Converged
        }
    }

    // Compute K = (R + B'PB)^(-1) * B'PA
    matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);
    matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

    matrixAdd(R, temp2, temp2, controlSize, controlSize);

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

    matrixMultiply(temp1, A, temp3, controlSize, stateSize, stateSize);
    matrixMultiply(temp2, temp3, K, controlSize, controlSize, stateSize);

    // Clean up memory
    delete[] P_next;
    delete[] BT;
    delete[] AT;
    delete[] temp1;
    delete[] temp2;
    delete[] temp3;
    delete[] temp4;
    delete[] temp5;

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
