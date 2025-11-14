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

// ============================================================================
// FUNÇÕES AUXILIARES PARA DECOMPOSIÇÃO QZ/SCHUR
// ============================================================================

// Rotação de Givens para zerar elemento (i+1, i)
IRAM_ATTR void MatrixOperations::givensRotation(float* matrix, int n, int i, int j, float c, float s)
{
    if (!matrix || i >= n || j >= n)
        return;
    
    // Aplica rotação nas linhas i e j
    for (int k = 0; k < n; k++) {
        float temp_i = matrix[i * n + k];
        float temp_j = matrix[j * n + k];
        matrix[i * n + k] = c * temp_i - s * temp_j;
        matrix[j * n + k] = s * temp_i + c * temp_j;
    }
}

// Redução para forma de Hessenberg (quase triangular superior)
IRAM_ATTR bool MatrixOperations::hessenbergReduction(float* matrix, float* Q, int n)
{
    if (!matrix || !Q || n <= 2)
        return false;
    
    // Inicializar Q como identidade
    matrixIdentity(Q, n);
    
    // Para cada coluna (exceto as duas últimas)
    for (int k = 0; k < n - 2; k++) {
        // Encontrar elemento máximo abaixo da diagonal
        float max_val = fabsf(matrix[(k + 1) * n + k]);
        int max_idx = k + 1;
        
        for (int i = k + 2; i < n; i++) {
            float val = fabsf(matrix[i * n + k]);
            if (val > max_val) {
                max_val = val;
                max_idx = i;
            }
        }
        
        // Se elemento já é zero, continuar
        if (max_val < EPSILON)
            continue;
        
        // Calcular rotação de Givens
        float a = matrix[(k + 1) * n + k];
        float b = matrix[max_idx * n + k];
        float r = sqrtf(a * a + b * b);
        float c = a / r;
        float s = -b / r;
        
        // Aplicar rotação em matrix e Q
        givensRotation(matrix, n, k + 1, max_idx, c, s);
        givensRotation(Q, n, k + 1, max_idx, c, s);
        
        // Aplicar rotação transposta nas colunas
        for (int i = 0; i < n; i++) {
            float temp_k1 = matrix[i * n + (k + 1)];
            float temp_max = matrix[i * n + max_idx];
            matrix[i * n + (k + 1)] = c * temp_k1 - s * temp_max;
            matrix[i * n + max_idx] = s * temp_k1 + c * temp_max;
        }
    }
    
    return true;
}

// Decomposição QR usando rotações de Givens
IRAM_ATTR void MatrixOperations::qrDecomposition(float* matrix, float* Q, float* R, int n)
{
    if (!matrix || !Q || !R)
        return;
    
    // Copiar matrix para R
    matrixCopy(matrix, R, n * n);
    
    // Inicializar Q como identidade
    matrixIdentity(Q, n);
    
    // Aplicar rotações de Givens para triangularizar R
    for (int j = 0; j < n - 1; j++) {
        for (int i = n - 1; i > j; i--) {
            float a = R[(i - 1) * n + j];
            float b = R[i * n + j];
            
            if (fabsf(b) < EPSILON)
                continue;
            
            float r = sqrtf(a * a + b * b);
            float c = a / r;
            float s = -b / r;
            
            // Aplicar rotação em R
            for (int k = j; k < n; k++) {
                float temp_im1 = R[(i - 1) * n + k];
                float temp_i = R[i * n + k];
                R[(i - 1) * n + k] = c * temp_im1 - s * temp_i;
                R[i * n + k] = s * temp_im1 + c * temp_i;
            }
            
            // Aplicar rotação transposta em Q
            for (int k = 0; k < n; k++) {
                float temp_im1 = Q[k * n + (i - 1)];
                float temp_i = Q[k * n + i];
                Q[k * n + (i - 1)] = c * temp_im1 - s * temp_i;
                Q[k * n + i] = s * temp_im1 + c * temp_i;
            }
        }
    }
}

// Decomposição de Schur usando algoritmo QR iterativo
IRAM_ATTR bool MatrixOperations::schurDecomposition(float* matrix, float* Q, int n, int max_iterations)
{
    if (!matrix || !Q || n <= 0)
        return false;
    
    // Inicializar Q como identidade
    matrixIdentity(Q, n);
    
    // Reduzir para forma de Hessenberg primeiro (otimização)
    float* Q_hess = new float[n * n];
    if (!hessenbergReduction(matrix, Q_hess, n)) {
        delete[] Q_hess;
        return false;
    }
    
    // Atualizar Q
    float* temp_Q = new float[n * n];
    matrixMultiply(Q, Q_hess, temp_Q, n, n, n);
    matrixCopy(temp_Q, Q, n * n);
    
    // Alocar temporários para QR
    float* Q_iter = new float[n * n];
    float* R_iter = new float[n * n];
    
    // Iteração QR
    for (int iter = 0; iter < max_iterations; iter++) {
        // Verificar convergência (elemento sub-diagonal pequeno)
        bool converged = true;
        for (int i = 0; i < n - 1; i++) {
            if (fabsf(matrix[(i + 1) * n + i]) > EPSILON) {
                converged = false;
                break;
            }
        }
        
        if (converged)
            break;
        
        // QR decomposition: matrix = Q_iter * R_iter
        qrDecomposition(matrix, Q_iter, R_iter, n);
        
        // Atualizar matrix = R_iter * Q_iter
        matrixMultiply(R_iter, Q_iter, matrix, n, n, n);
        
        // Atualizar Q acumulado
        matrixMultiply(Q, Q_iter, temp_Q, n, n, n);
        matrixCopy(temp_Q, Q, n * n);
    }
    
    // Limpar memória
    delete[] Q_hess;
    delete[] temp_Q;
    delete[] Q_iter;
    delete[] R_iter;
    
    return true;
}

// Calcular autovalores de matriz usando QR
IRAM_ATTR bool MatrixOperations::computeEigenvalues(const float* matrix, float* eigenvalues_real,
                                                   float* eigenvalues_imag, int n, int max_iterations)
{
    if (!matrix || !eigenvalues_real || !eigenvalues_imag || n <= 0)
        return false;
    
    // Copiar matriz (será modificada)
    float* A = new float[n * n];
    float* Q = new float[n * n];
    matrixCopy(matrix, A, n * n);
    
    // Decomposição de Schur
    if (!schurDecomposition(A, Q, n, max_iterations)) {
        delete[] A;
        delete[] Q;
        return false;
    }
    
    // Extrair autovalores da matriz triangular superior (Schur form)
    for (int i = 0; i < n; i++) {
        eigenvalues_real[i] = A[i * n + i];
        eigenvalues_imag[i] = 0.0f;
        
        // Detectar pares complexos conjugados (blocos 2x2)
        if (i < n - 1 && fabsf(A[(i + 1) * n + i]) > EPSILON) {
            // Bloco 2x2: calcular autovalores complexos
            float a = A[i * n + i];
            float b = A[i * n + (i + 1)];
            float c = A[(i + 1) * n + i];
            float d = A[(i + 1) * n + (i + 1)];
            
            float trace = a + d;
            float det = a * d - b * c;
            float discriminant = trace * trace - 4.0f * det;
            
            if (discriminant < 0) {
                // Autovalores complexos conjugados
                eigenvalues_real[i] = eigenvalues_real[i + 1] = trace * 0.5f;
                eigenvalues_imag[i] = sqrtf(-discriminant) * 0.5f;
                eigenvalues_imag[i + 1] = -eigenvalues_imag[i];
                i++; // Pular próximo (par conjugado)
            }
        }
    }
    
    delete[] A;
    delete[] Q;
    return true;
}

// Resolver problema generalizado de autovalores para DARE
// Usa método simplificado específico para matrizes DARE
IRAM_ATTR bool MatrixOperations::solveGeneralizedEigenproblemDARE(const float* H, const float* J,
                                                                  float* stable_eigenvectors, int n)
{
    if (!H || !J || !stable_eigenvectors || n <= 0)
        return false;
    
    const int n2 = 2 * n;
    
    // Para DARE, podemos simplificar: resolver J^{-1} * H
    // (válido porque J é inversível para problemas bem-postos)
    
    // Alocar memória
    float* J_inv = new float[n2 * n2];
    float* J_inv_H = new float[n2 * n2];
    float* eigenvalues_real = new float[n2];
    float* eigenvalues_imag = new float[n2];
    float* eigenvectors = new float[n2 * n2];
    
    // Inverter J (usando Gauss-Jordan otimizado)
    matrixCopy(J, J_inv, n2 * n2);
    
    // Para matrizes grandes, usar método iterativo
    // Por simplicidade, vamos usar abordagem direta para n pequeno (4x4 -> 8x8)
    if (n2 == 4 || n2 == 6 || n2 == 8) {
        // Gauss-Jordan para inversão
        float* augmented = new float[n2 * 2 * n2];
        matrixClear(augmented, n2 * 2 * n2);
        
        // [J | I]
        for (int i = 0; i < n2; i++) {
            for (int j = 0; j < n2; j++) {
                augmented[i * 2 * n2 + j] = J[i * n2 + j];
            }
            augmented[i * 2 * n2 + (i + n2)] = 1.0f;
        }
        
        // Eliminação de Gauss-Jordan
        for (int i = 0; i < n2; i++) {
            // Pivotamento parcial
            float max_val = fabsf(augmented[i * 2 * n2 + i]);
            int max_row = i;
            for (int k = i + 1; k < n2; k++) {
                float val = fabsf(augmented[k * 2 * n2 + i]);
                if (val > max_val) {
                    max_val = val;
                    max_row = k;
                }
            }
            
            if (max_val < EPSILON) {
                delete[] J_inv; delete[] J_inv_H; delete[] eigenvalues_real;
                delete[] eigenvalues_imag; delete[] eigenvectors; delete[] augmented;
                return false;
            }
            
            // Trocar linhas
            if (max_row != i) {
                for (int j = 0; j < 2 * n2; j++) {
                    float temp = augmented[i * 2 * n2 + j];
                    augmented[i * 2 * n2 + j] = augmented[max_row * 2 * n2 + j];
                    augmented[max_row * 2 * n2 + j] = temp;
                }
            }
            
            // Normalizar linha
            float pivot = augmented[i * 2 * n2 + i];
            for (int j = 0; j < 2 * n2; j++) {
                augmented[i * 2 * n2 + j] /= pivot;
            }
            
            // Eliminar coluna em outras linhas
            for (int k = 0; k < n2; k++) {
                if (k != i) {
                    float factor = augmented[k * 2 * n2 + i];
                    for (int j = 0; j < 2 * n2; j++) {
                        augmented[k * 2 * n2 + j] -= factor * augmented[i * 2 * n2 + j];
                    }
                }
            }
        }
        
        // Extrair J_inv
        for (int i = 0; i < n2; i++) {
            for (int j = 0; j < n2; j++) {
                J_inv[i * n2 + j] = augmented[i * 2 * n2 + (j + n2)];
            }
        }
        
        delete[] augmented;
    } else {
        delete[] J_inv; delete[] J_inv_H; delete[] eigenvalues_real;
        delete[] eigenvalues_imag; delete[] eigenvectors;
        return false;
    }
    
    // Calcular J^{-1} * H
    matrixMultiply(J_inv, H, J_inv_H, n2, n2, n2);
    
    // Calcular autovalores e autovetores
    if (!computeEigenvalues(J_inv_H, eigenvalues_real, eigenvalues_imag, n2)) {
        delete[] J_inv; delete[] J_inv_H; delete[] eigenvalues_real;
        delete[] eigenvalues_imag; delete[] eigenvectors;
        return false;
    }
    
    // Identificar autovalores estáveis (|λ| < 1)
    int stable_count = 0;
    int stable_indices[n2];
    
    for (int i = 0; i < n2; i++) {
        float magnitude = sqrtf(eigenvalues_real[i] * eigenvalues_real[i] +
                               eigenvalues_imag[i] * eigenvalues_imag[i]);
        if (magnitude < 1.0f) {
            stable_indices[stable_count++] = i;
        }
    }
    
    if (stable_count != n) {
        Serial.print(F("AVISO: Encontrados "));
        Serial.print(stable_count);
        Serial.print(F(" autovalores estáveis, esperados "));
        Serial.println(n);
        
        delete[] J_inv; delete[] J_inv_H; delete[] eigenvalues_real;
        delete[] eigenvalues_imag; delete[] eigenvectors;
        return false;
    }
    
    // TODO: Calcular autovetores correspondentes
    // Por simplicidade, usar power iteration para cada autovalor estável
    // (implementação completa requer mais trabalho)
    
    // Por enquanto, retornar erro indicando que precisa de implementação completa
    Serial.println(F("AVISO: Cálculo de autovetores ainda não implementado"));
    
    delete[] J_inv;
    delete[] J_inv_H;
    delete[] eigenvalues_real;
    delete[] eigenvalues_imag;
    delete[] eigenvectors;
    
    return false; // Por enquanto, retornar false até implementar autovetores
}
