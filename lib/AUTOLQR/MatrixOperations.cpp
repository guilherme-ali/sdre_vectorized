#include "MatrixOperations.h"
#include <math.h>
#include <string.h>
#include <new>

// Otimizações específicas para ESP32
#if defined(ESP32)
    #include "esp_attr.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#endif

// Macro para alinhamento de memória otimizado
#define ALIGN_32 __attribute__((aligned(32)))

// Função auxiliar para multiplicação de matrizes pequenas otimizada
IRAM_ATTR static inline void multiply_2x2(const float* a, const float* b, float* c) {
    // Otimização para 2x2 usando registradores
    register float a00 = a[0], a01 = a[1], a10 = a[2], a11 = a[3];
    register float b00 = b[0], b01 = b[1], b10 = b[2], b11 = b[3];
    
    c[0] = a00 * b00 + a01 * b10;
    c[1] = a00 * b01 + a01 * b11;
    c[2] = a10 * b00 + a11 * b10;
    c[3] = a10 * b01 + a11 * b11;
}

IRAM_ATTR static inline void multiply_3x3(const float* a, const float* b, float* c) {
    // Otimização para 3x3 usando registradores e desenrolamento
    register float a00 = a[0], a01 = a[1], a02 = a[2];
    register float a10 = a[3], a11 = a[4], a12 = a[5];
    register float a20 = a[6], a21 = a[7], a22 = a[8];
    
    register float b00 = b[0], b10 = b[3], b20 = b[6];
    register float b01 = b[1], b11 = b[4], b21 = b[7];
    register float b02 = b[2], b12 = b[5], b22 = b[8];
    
    c[0] = a00 * b00 + a01 * b10 + a02 * b20;
    c[1] = a00 * b01 + a01 * b11 + a02 * b21;
    c[2] = a00 * b02 + a01 * b12 + a02 * b22;
    c[3] = a10 * b00 + a11 * b10 + a12 * b20;
    c[4] = a10 * b01 + a11 * b11 + a12 * b21;
    c[5] = a10 * b02 + a11 * b12 + a12 * b22;
    c[6] = a20 * b00 + a21 * b10 + a22 * b20;
    c[7] = a20 * b01 + a21 * b11 + a22 * b21;
    c[8] = a20 * b02 + a21 * b12 + a22 * b22;
}

IRAM_ATTR void MatrixOperations::matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2)
{
    if (!m1 || !m2 || !result)
        return;

    // Otimizações para tamanhos específicos comuns
    if (rows1 == 2 && cols1 == 2 && cols2 == 2) {
        multiply_2x2(m1, m2, result);
        return;
    }
    
    if (rows1 == 3 && cols1 == 3 && cols2 == 3) {
        multiply_3x3(m1, m2, result);
        return;
    }

    // Limpar resultado usando memset (mais rápido)
    memset(result, 0, rows1 * cols2 * sizeof(float));

    // Otimização de cache: trocar ordem dos loops para melhor localidade
    // e desenrolar loops internos quando possível
    for (int i = 0; i < rows1; i++) {
        for (int k = 0; k < cols1; k++) {
            register float a_ik = m1[i * cols1 + k];
            if (a_ik != 0.0f) { // Pular multiplicações por zero
                const float* b_row = &m2[k * cols2];
                float* c_row = &result[i * cols2];
                
                // Desenrolar loop interno para múltiplos de 4
                int j = 0;
                for (; j < (cols2 & ~3); j += 4) {
                    c_row[j] += a_ik * b_row[j];
                    c_row[j+1] += a_ik * b_row[j+1];
                    c_row[j+2] += a_ik * b_row[j+2];
                    c_row[j+3] += a_ik * b_row[j+3];
                }
                // Processar elementos restantes
                for (; j < cols2; j++) {
                    c_row[j] += a_ik * b_row[j];
                }
            }
        }
    }
}

IRAM_ATTR void MatrixOperations::matrixAdd(const float* m1, const float* m2, float* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    int total = rows * cols;
    int i = 0;
    
    // Desenrolar loop para múltiplos de 8 (otimização vetorial)
    for (; i < (total & ~7); i += 8) {
        result[i] = m1[i] + m2[i];
        result[i+1] = m1[i+1] + m2[i+1];
        result[i+2] = m1[i+2] + m2[i+2];
        result[i+3] = m1[i+3] + m2[i+3];
        result[i+4] = m1[i+4] + m2[i+4];
        result[i+5] = m1[i+5] + m2[i+5];
        result[i+6] = m1[i+6] + m2[i+6];
        result[i+7] = m1[i+7] + m2[i+7];
    }
    
    // Processar elementos restantes
    for (; i < total; i++) {
        result[i] = m1[i] + m2[i];
    }
}

IRAM_ATTR void MatrixOperations::matrixSubtract(const float* m1, const float* m2, float* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    int total = rows * cols;
    int i = 0;
    
    // Desenrolar loop para múltiplos de 8
    for (; i < (total & ~7); i += 8) {
        result[i] = m1[i] - m2[i];
        result[i+1] = m1[i+1] - m2[i+1];
        result[i+2] = m1[i+2] - m2[i+2];
        result[i+3] = m1[i+3] - m2[i+3];
        result[i+4] = m1[i+4] - m2[i+4];
        result[i+5] = m1[i+5] - m2[i+5];
        result[i+6] = m1[i+6] - m2[i+6];
        result[i+7] = m1[i+7] - m2[i+7];
    }
    
    // Processar elementos restantes
    for (; i < total; i++) {
        result[i] = m1[i] - m2[i];
    }
}

IRAM_ATTR void MatrixOperations::transposeMatrix(const float* matrix, float* result, int rows, int cols)
{
    if (!matrix || !result)
        return;

    // Otimização para matrizes pequenas comuns
    if (rows == 2 && cols == 2) {
        result[0] = matrix[0]; result[1] = matrix[2];
        result[2] = matrix[1]; result[3] = matrix[3];
        return;
    }
    
    if (rows == 3 && cols == 3) {
        result[0] = matrix[0]; result[1] = matrix[3]; result[2] = matrix[6];
        result[3] = matrix[1]; result[4] = matrix[4]; result[5] = matrix[7];
        result[6] = matrix[2]; result[7] = matrix[5]; result[8] = matrix[8];
        return;
    }

    // Transposição por blocos para melhor localidade de cache
    const int BLOCK_SIZE = 4;
    
    for (int i = 0; i < rows; i += BLOCK_SIZE) {
        for (int j = 0; j < cols; j += BLOCK_SIZE) {
            int max_i = (i + BLOCK_SIZE < rows) ? i + BLOCK_SIZE : rows;
            int max_j = (j + BLOCK_SIZE < cols) ? j + BLOCK_SIZE : cols;
            
            for (int ii = i; ii < max_i; ii++) {
                for (int jj = j; jj < max_j; jj++) {
                    result[jj * rows + ii] = matrix[ii * cols + jj];
                }
            }
        }
    }
}

// Otimização específica para inversão 2x2
IRAM_ATTR static inline bool invert_2x2(const float* matrix, float* result) {
    register float a = matrix[0], b = matrix[1], c = matrix[2], d = matrix[3];
    register float det = a * d - b * c;
    
    if (fabsf(det) < 1e-6f)
        return false;
    
    register float inv_det = 1.0f / det;
    result[0] = d * inv_det;
    result[1] = -b * inv_det;
    result[2] = -c * inv_det;
    result[3] = a * inv_det;
    
    return true;
}

// Otimização específica para inversão 3x3
IRAM_ATTR static inline bool invert_3x3(const float* matrix, float* result) {
    register float a = matrix[0], b = matrix[1], c = matrix[2];
    register float d = matrix[3], e = matrix[4], f = matrix[5];
    register float g = matrix[6], h = matrix[7], i = matrix[8];

    register float ei_fh = e * i - f * h;
    register float ch_bi = c * h - b * i;
    register float bf_ce = b * f - c * e;
    
    register float det = a * ei_fh + b * (f * g - d * i) + c * (d * h - e * g);
    
    if (fabsf(det) < 1e-6f)
        return false;

    register float inv_det = 1.0f / det;

    result[0] = ei_fh * inv_det;
    result[1] = ch_bi * inv_det;
    result[2] = bf_ce * inv_det;
    result[3] = (f * g - d * i) * inv_det;
    result[4] = (a * i - c * g) * inv_det;
    result[5] = (c * d - a * f) * inv_det;
    result[6] = (d * h - e * g) * inv_det;
    result[7] = (b * g - a * h) * inv_det;
    result[8] = (a * e - b * d) * inv_det;

    return true;
}

bool MatrixOperations::invertMatrix(const float* matrix, float* result, int n)
{
    if (!matrix || !result)
        return false;

    if (n == 1) {
        if (fabsf(matrix[0]) < 1e-6f)
            return false;
        result[0] = 1.0f / matrix[0];
        return true;
    } 
    else if (n == 2) {
        return invert_2x2(matrix, result);
    } 
    else if (n == 3) {
        return invert_3x3(matrix, result);
    } 
    else if (n == 4) {
        // Gauss-Jordan otimizado para 4x4
        ALIGN_32 float augmented[4 * 8];
        memset(augmented, 0, sizeof(augmented));

        // Inicializar matriz aumentada [A|I]
        for (int i = 0; i < 4; i++) {
            memcpy(&augmented[i * 8], &matrix[i * 4], 4 * sizeof(float));
            augmented[i * 8 + (i + 4)] = 1.0f;
        }

        // Eliminação de Gauss-Jordan otimizada
        for (int i = 0; i < 4; i++) {
            // Encontrar pivô
            float max_val = fabsf(augmented[i * 8 + i]);
            int max_row = i;
            
            for (int j = i + 1; j < 4; j++) {
                float val = fabsf(augmented[j * 8 + i]);
                if (val > max_val) {
                    max_val = val;
                    max_row = j;
                }
            }

            if (max_val < 1e-6f)
                return false;

            // Trocar linhas se necessário
            if (max_row != i) {
                for (int j = 0; j < 8; j++) {
                    float temp = augmented[i * 8 + j];
                    augmented[i * 8 + j] = augmented[max_row * 8 + j];
                    augmented[max_row * 8 + j] = temp;
                }
            }

            // Escalar linha para que o pivô seja 1
            float pivot_val = augmented[i * 8 + i];
            float inv_pivot = 1.0f / pivot_val;
            
            for (int j = 0; j < 8; j++) {
                augmented[i * 8 + j] *= inv_pivot;
            }

            // Eliminar outras linhas
            for (int j = 0; j < 4; j++) {
                if (j != i) {
                    float factor = augmented[j * 8 + i];
                    for (int k = 0; k < 8; k++) {
                        augmented[j * 8 + k] -= factor * augmented[i * 8 + k];
                    }
                }
            }
        }

        // Extrair inversa
        for (int i = 0; i < 4; i++) {
            memcpy(&result[i * 4], &augmented[i * 8 + 4], 4 * sizeof(float));
        }

        return true;
    }
    else if (n == 6) {
        // Gauss-Jordan otimizado para 6x6
        ALIGN_32 float* augmented = new(std::nothrow) float[6 * 12];
        if (!augmented) return false;
        
        memset(augmented, 0, 6 * 12 * sizeof(float));
        
        // Inicializar matriz aumentada [A|I]
        for (int i = 0; i < 6; i++) {
            memcpy(&augmented[i * 12], &matrix[i * 6], 6 * sizeof(float));
            augmented[i * 12 + (i + 6)] = 1.0f;
        }
        
        // Eliminação de Gauss-Jordan
        for (int i = 0; i < 6; i++) {
            // Encontrar pivô
            float max_val = fabsf(augmented[i * 12 + i]);
            int max_row = i;
            
            for (int j = i + 1; j < 6; j++) {
                float val = fabsf(augmented[j * 12 + i]);
                if (val > max_val) {
                    max_val = val;
                    max_row = j;
                }
            }
            
            if (max_val < 1e-6f) {
                delete[] augmented;
                return false;
            }
                
            // Trocar linhas se necessário
            if (max_row != i) {
                for (int j = 0; j < 12; j++) {
                    float temp = augmented[i * 12 + j];
                    augmented[i * 12 + j] = augmented[max_row * 12 + j];
                    augmented[max_row * 12 + j] = temp;
                }
            }
            
            // Escalar linha
            float pivot_val = augmented[i * 12 + i];
            float inv_pivot = 1.0f / pivot_val;
            
            for (int j = 0; j < 12; j++) {
                augmented[i * 12 + j] *= inv_pivot;
            }
            
            // Eliminar outras linhas
            for (int j = 0; j < 6; j++) {
                if (j != i) {
                    float factor = augmented[j * 12 + i];
                    for (int k = 0; k < 12; k++) {
                        augmented[j * 12 + k] -= factor * augmented[i * 12 + k];
                    }
                }
            }
        }
        
        // Extrair inversa
        for (int i = 0; i < 6; i++) {
            memcpy(&result[i * 6], &augmented[i * 12 + 6], 6 * sizeof(float));
        }
        
        delete[] augmented;
        return true;
    }
    
    Serial.print("Tamanho da matriz não suportado para inversão, tamanho: ");
    Serial.println(n);
    return false;
}

IRAM_ATTR void MatrixOperations::matrixCopy(const float* source, float* dest, int size)
{
    if (!source || !dest)
        return;
    
    // memcpy é otimizado para ESP32
    memcpy(dest, source, sizeof(float) * size);
}

IRAM_ATTR void MatrixOperations::matrixClear(float* matrix, int size)
{
    if (!matrix)
        return;
    
    // memset é mais rápido que loop
    memset(matrix, 0, sizeof(float) * size);
}

IRAM_ATTR void MatrixOperations::matrixIdentity(float* matrix, int n)
{
    if (!matrix)
        return;
    
    memset(matrix, 0, n * n * sizeof(float));
    
    // Desenrolar para tamanhos comuns
    if (n == 2) {
        matrix[0] = 1.0f;
        matrix[3] = 1.0f;
    } else if (n == 3) {
        matrix[0] = 1.0f;
        matrix[4] = 1.0f;
        matrix[8] = 1.0f;
    } else if (n == 4) {
        matrix[0] = 1.0f;
        matrix[5] = 1.0f;
        matrix[10] = 1.0f;
        matrix[15] = 1.0f;
    } else {
        // Caso geral
        for (int i = 0; i < n; i++) {
            matrix[i * n + i] = 1.0f;
        }
    }
}

IRAM_ATTR void MatrixOperations::printMatrix(const float* matrix, int rows, int cols, const char* name)
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

IRAM_ATTR float MatrixOperations::det2x2(const float* matrix)
{
    if (!matrix)
        return 0.0f;
    return matrix[0] * matrix[3] - matrix[1] * matrix[2];
}

IRAM_ATTR float MatrixOperations::det3x3(const float* matrix)
{
    if (!matrix)
        return 0.0f;
    return matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
           matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
           matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
}

IRAM_ATTR float MatrixOperations::dotProduct(const float* v1, const float* v2, int size)
{
    if (!v1 || !v2)
        return 0.0f;
        
    float result = 0.0f;
    int i = 0;
    
    // Desenrolar para múltiplos de 4
    for (; i < (size & ~3); i += 4) {
        result += v1[i] * v2[i] + v1[i+1] * v2[i+1] + 
                  v1[i+2] * v2[i+2] + v1[i+3] * v2[i+3];
    }
    
    // Processar elementos restantes
    for (; i < size; i++) {
        result += v1[i] * v2[i];
    }
    
    return result;
}

IRAM_ATTR void MatrixOperations::matrixVectorMultiply(const float* matrix, const float* vector, 
                                                     float* result, int rows, int cols)
{
    if (!matrix || !vector || !result)
        return;
        
    for (int i = 0; i < rows; i++) {
        float sum = 0.0f;
        const float* row = &matrix[i * cols];
        
        // Desenrolar para múltiplos de 4
        int j = 0;
        for (; j < (cols & ~3); j += 4) {
            sum += row[j] * vector[j] + row[j+1] * vector[j+1] + 
                   row[j+2] * vector[j+2] + row[j+3] * vector[j+3];
        }
        
        // Processar elementos restantes
        for (; j < cols; j++) {
            sum += row[j] * vector[j];
        }
        
        result[i] = sum;
    }
}