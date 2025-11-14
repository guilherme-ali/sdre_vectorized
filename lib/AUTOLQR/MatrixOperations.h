#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

#include <Arduino.h>

// Definições de otimização para ESP32
#if defined(ESP32)
    #define MATRIX_INLINE __attribute__((always_inline)) inline
    #define MATRIX_FAST_ATTR IRAM_ATTR
    #define MATRIX_ALIGN __attribute__((aligned(32)))
#else
    #define MATRIX_INLINE inline
    #define MATRIX_FAST_ATTR
    #define MATRIX_ALIGN
#endif

class MatrixOperations {
public:
    /**
     * @brief Multiply two matrices (otimizado para ESP32)
     * @param m1 First matrix
     * @param m2 Second matrix  
     * @param result Result matrix
     * @param rows1 Rows in first matrix
     * @param cols1 Columns in first matrix / Rows in second matrix
     * @param cols2 Columns in second matrix
     * @note Otimizado para matrizes 2x2 e 3x3, com desenrolamento de loops
     */
    static void matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2);

    /**
     * @brief Add two matrices (otimizado com desenrolamento de loops)
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows Rows in matrices
     * @param cols Columns in matrices
     * @note Utiliza desenrolamento de loops para melhor performance
     */
    static void matrixAdd(const float* m1, const float* m2, float* result, int rows, int cols);

    /**
     * @brief Subtract second matrix from first (otimizado)
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows Rows in matrices
     * @param cols Columns in matrices
     * @note Utiliza desenrolamento de loops vetorizados
     */
    static void matrixSubtract(const float* m1, const float* m2, float* result, int rows, int cols);

    /**
     * @brief Transpose a matrix (otimizado com cache blocking)
     * @param matrix Input matrix
     * @param result Transposed matrix
     * @param rows Rows in input matrix
     * @param cols Columns in input matrix
     * @note Usa transposição por blocos para melhor localidade de cache
     */
    static void transposeMatrix(const float* matrix, float* result, int rows, int cols);

    /**
     * @brief Invert a matrix (altamente otimizado para tamanhos específicos)
     * @param matrix Input matrix
     * @param result Inverted matrix
     * @param n Matrix size (suporta 1x1, 2x2, 3x3, 4x4, 6x6)
     * @return true if successful, false if matrix is singular
     * @note Implementações especializadas para cada tamanho suportado
     */
    static bool invertMatrix(const float* matrix, float* result, int n);

    /**
     * @brief Copy matrix content (otimizado com memcpy)
     * @param source Source matrix
     * @param dest Destination matrix
     * @param size Number of elements to copy
     * @note Usa memcpy otimizado do ESP32
     */
    static void matrixCopy(const float* source, float* dest, int size);

    /**
     * @brief Set matrix to zero (otimizado com memset)
     * @param matrix Matrix to clear
     * @param size Number of elements
     * @note Usa memset para alta performance
     */
    static void matrixClear(float* matrix, int size);

    /**
     * @brief Set matrix to identity (otimizado para tamanhos comuns)
     * @param matrix Matrix to set as identity
     * @param n Matrix dimension (n x n)
     * @note Desenrolado para matrizes 2x2, 3x3 e 4x4
     */
    static void matrixIdentity(float* matrix, int n);

    /**
     * @brief Print matrix for debugging
     * @param matrix Matrix to print
     * @param rows Number of rows
     * @param cols Number of columns
     * @param name Optional name for the matrix
     */
    static void printMatrix(const float* matrix, int rows, int cols, const char* name = nullptr);

    // Funções auxiliares inline para operações rápidas
    
    /**
     * @brief Fast 2x2 matrix determinant
     * @param matrix 2x2 matrix
     * @return determinant value
     */
    static float det2x2(const float* matrix);

    /**
     * @brief Fast 3x3 matrix determinant
     * @param matrix 3x3 matrix  
     * @return determinant value
     */
    static float det3x3(const float* matrix);

    /**
     * @brief Fast vector dot product
     * @param v1 First vector
     * @param v2 Second vector
     * @param size Vector size
     * @return dot product result
     */
    static float dotProduct(const float* v1, const float* v2, int size);

    /**
     * @brief Fast matrix-vector multiplication
     * @param matrix Matrix (rows x cols)
     * @param vector Vector (cols elements)
     * @param result Result vector (rows elements)
     * @param rows Matrix rows
     * @param cols Matrix columns
     */
    static void matrixVectorMultiply(const float* matrix, const float* vector, 
                                    float* result, int rows, int cols);

    /**
     * @brief Compute eigenvalues using QR algorithm for DARE
     * @param matrix Input matrix (n x n)
     * @param eigenvalues_real Real parts of eigenvalues (n elements)
     * @param eigenvalues_imag Imaginary parts of eigenvalues (n elements)
     * @param n Matrix dimension
     * @param max_iterations Maximum QR iterations
     * @return true if successful
     */
    static bool computeEigenvalues(const float* matrix, float* eigenvalues_real, 
                                  float* eigenvalues_imag, int n, int max_iterations = 100);

    /**
     * @brief Simplified QZ decomposition for DARE problem
     * Computes stable eigenspace for H and J matrices in DARE
     * @param H First matrix (2n x 2n)
     * @param J Second matrix (2n x 2n)
     * @param stable_eigenvectors Output stable eigenvectors (2n x n)
     * @param n Half dimension (state size)
     * @return true if successful
     */
    static bool solveGeneralizedEigenproblemDARE(const float* H, const float* J,
                                                  float* stable_eigenvectors, int n);

    /**
     * @brief Fast Schur decomposition using QR algorithm
     * @param matrix Input matrix (n x n) - will be modified to upper triangular form
     * @param Q Orthogonal matrix (n x n)
     * @param n Matrix dimension
     * @param max_iterations Maximum iterations
     * @return true if successful
     */
    static bool schurDecomposition(float* matrix, float* Q, int n, int max_iterations = 200);

private:
    // Constantes para otimização
    static const int CACHE_LINE_SIZE = 32;
    static const int BLOCK_SIZE = 4;
    static constexpr float EPSILON = 1e-6f;
    
    // Funções auxiliares para decomposição QZ
    static void qrDecomposition(float* matrix, float* Q, float* R, int n);
    static void givensRotation(float* matrix, int n, int i, int j, float c, float s);
    static bool hessenbergReduction(float* matrix, float* Q, int n);
};

#endif