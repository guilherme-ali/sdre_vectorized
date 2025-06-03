#include "MatrixOperations.h"
#include <math.h>

void MatrixOperations::matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2)
{
    if (!m1 || !m2 || !result)
        return;

    // Initialize result matrix to zero
    for (int i = 0; i < rows1 * cols2; i++) {
        result[i] = 0;
    }

    for (int i = 0; i < rows1; i++) {
        for (int j = 0; j < cols2; j++) {
            for (int k = 0; k < cols1; k++) {
                result[i * cols2 + j] += m1[i * cols1 + k] * m2[k * cols2 + j];
            }
        }
    }
}

void MatrixOperations::matrixAdd(const float* m1, const float* m2, float* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    for (int i = 0; i < rows * cols; i++) {
        result[i] = m1[i] + m2[i];
    }
}

void MatrixOperations::matrixSubtract(const float* m1, const float* m2, float* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    for (int i = 0; i < rows * cols; i++) {
        result[i] = m1[i] - m2[i];
    }
}

void MatrixOperations::transposeMatrix(const float* matrix, float* result, int rows, int cols)
{
    if (!matrix || !result)
        return;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[j * rows + i] = matrix[i * cols + j];
        }
    }
}

bool MatrixOperations::invertMatrix(const float* matrix, float* result, int n)
{
    if (!matrix || !result)
        return false;

    if (n == 1) {
        if (fabs(matrix[0]) < 1e-6)
            return false;
        result[0] = 1.0f / matrix[0];
        return true;
    } else if (n == 2) {
        float det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
        if (fabs(det) < 1e-6)
            return false;
        float invDet = 1.0f / det;
        result[0] = matrix[3] * invDet;
        result[1] = -matrix[1] * invDet;
        result[2] = -matrix[2] * invDet;
        result[3] = matrix[0] * invDet;
        return true;
    } else if (n == 3) {
        // 3x3 Matrix inversion
        float a = matrix[0], b = matrix[1], c = matrix[2];
        float d = matrix[3], e = matrix[4], f = matrix[5];
        float g = matrix[6], h = matrix[7], i = matrix[8];

        float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if (fabs(det) < 1e-6)
            return false;

        float invDet = 1.0f / det;

        result[0] = (e * i - f * h) * invDet;
        result[1] = (c * h - b * i) * invDet;
        result[2] = (b * f - c * e) * invDet;
        result[3] = (f * g - d * i) * invDet;
        result[4] = (a * i - c * g) * invDet;
        result[5] = (c * d - a * f) * invDet;
        result[6] = (d * h - e * g) * invDet;
        result[7] = (b * g - a * h) * invDet;
        result[8] = (a * e - b * d) * invDet;

        return true;
    } else if (n == 4) {
        // 4x4 Matrix inversion using Gauss-Jordan elimination
        float* augmented = new float[4 * 8](); // Augmented matrix [A|I]

        // Initialize augmented matrix with [A|I]
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                augmented[i * 8 + j] = matrix[i * 4 + j];
            }
            // Add identity matrix part
            augmented[i * 8 + (i + 4)] = 1.0f;
        }

        // Perform Gauss-Jordan elimination
        for (int i = 0; i < 4; i++) {
            // Find pivot
            float maxVal = fabs(augmented[i * 8 + i]);
            int maxRow = i;
            for (int j = i + 1; j < 4; j++) {
                if (fabs(augmented[j * 8 + i]) > maxVal) {
                    maxVal = fabs(augmented[j * 8 + i]);
                    maxRow = j;
                }
            }

            // Check for singular matrix
            if (maxVal < 1e-6) {
                delete[] augmented;
                return false;
            }

            // Swap rows if needed
            if (maxRow != i) {
                for (int j = 0; j < 8; j++) {
                    float temp = augmented[i * 8 + j];
                    augmented[i * 8 + j] = augmented[maxRow * 8 + j];
                    augmented[maxRow * 8 + j] = temp;
                }
            }

            // Scale row so pivot is 1
            float pivotVal = augmented[i * 8 + i];
            for (int j = 0; j < 8; j++) {
                augmented[i * 8 + j] /= pivotVal;
            }

            // Eliminate other rows
            for (int j = 0; j < 4; j++) {
                if (j != i) {
                    float factor = augmented[j * 8 + i];
                    for (int k = 0; k < 8; k++) {
                        augmented[j * 8 + k] -= factor * augmented[i * 8 + k];
                    }
                }
            }
        }

        // Extract inverse from augmented matrix
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result[i * 4 + j] = augmented[i * 8 + (j + 4)];
            }
        }

        delete[] augmented;
        return true;
    }
    else if (n == 6) {
        // Create augmented matrix [A|I]
        float* augmented = new float[6 * 12]();
        
        // Initialize augmented matrix with [A|I]
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                augmented[i * 12 + j] = matrix[i * 6 + j];
            }
            // Add identity matrix part
            augmented[i * 12 + (i + 6)] = 1.0f;
        }
        
        // Perform Gauss-Jordan elimination
        for (int i = 0; i < 6; i++) {
            // Find pivot
            float maxVal = fabs(augmented[i * 12 + i]);
            int maxRow = i;
            for (int j = i + 1; j < 6; j++) {
                if (fabs(augmented[j * 12 + i]) > maxVal) {
                    maxVal = fabs(augmented[j * 12 + i]);
                    maxRow = j;
                }
            }
            
            // Check for singular matrix
            if (maxVal < 1e-6) {
                delete[] augmented;
                return false;
            }
                
            // Swap rows if needed
            if (maxRow != i) {
                for (int j = 0; j < 12; j++) {
                    float temp = augmented[i * 12 + j];
                    augmented[i * 12 + j] = augmented[maxRow * 12 + j];
                    augmented[maxRow * 12 + j] = temp;
                }
            }
            
            // Scale row so pivot is 1
            float pivotVal = augmented[i * 12 + i];
            for (int j = 0; j < 12; j++) {
                augmented[i * 12 + j] /= pivotVal;
            }
            
            // Eliminate other rows
            for (int j = 0; j < 6; j++) {
                if (j != i) {
                    float factor = augmented[j * 12 + i];
                    for (int k = 0; k < 12; k++) {
                        augmented[j * 12 + k] -= factor * augmented[i * 12 + k];
                    }
                }
            }
        }
        
        // Extract inverse from augmented matrix
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                result[i * 6 + j] = augmented[i * 12 + (j + 6)];
            }
        }
        
        delete[] augmented;
        return true;
    }
    Serial.print("Tamanho da matriz não suportado para inversão, tamanho: ");
    Serial.println(n);
    return false;
}

void MatrixOperations::matrixCopy(const float* source, float* dest, int size)
{
    if (!source || !dest)
        return;
    
    memcpy(dest, source, sizeof(float) * size);
}

void MatrixOperations::matrixClear(float* matrix, int size)
{
    if (!matrix)
        return;
    
    for (int i = 0; i < size; i++) {
        matrix[i] = 0.0f;
    }
}

void MatrixOperations::matrixIdentity(float* matrix, int n)
{
    if (!matrix)
        return;
    
    matrixClear(matrix, n * n);
    for (int i = 0; i < n; i++) {
        matrix[i * n + i] = 1.0f;
    }
}

void MatrixOperations::printMatrix(const float* matrix, int rows, int cols, const char* name)
{
    if (!matrix)
        return;
    
    if (name) {
        Serial.print("Matrix ");
        Serial.print(name);
        Serial.println(":");
    }
    
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            Serial.print(matrix[i * cols + j], 4);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}