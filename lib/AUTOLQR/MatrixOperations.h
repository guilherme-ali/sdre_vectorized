#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

#include <Arduino.h>

class MatrixOperations {
public:
    /**
     * @brief Multiply two matrices
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows1 Rows in first matrix
     * @param cols1 Columns in first matrix / Rows in second matrix
     * @param cols2 Columns in second matrix
     */
    static void matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2);

    /**
     * @brief Add two matrices
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows Rows in matrices
     * @param cols Columns in matrices
     */
    static void matrixAdd(const float* m1, const float* m2, float* result, int rows, int cols);

    /**
     * @brief Subtract second matrix from first
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows Rows in matrices
     * @param cols Columns in matrices
     */
    static void matrixSubtract(const float* m1, const float* m2, float* result, int rows, int cols);

    /**
     * @brief Transpose a matrix
     * @param matrix Input matrix
     * @param result Transposed matrix
     * @param rows Rows in input matrix
     * @param cols Columns in input matrix
     */
    static void transposeMatrix(const float* matrix, float* result, int rows, int cols);

    /**
     * @brief Invert a small matrix (1x1, 2x2, 3x3, 4x4, 6x6)
     * @param matrix Input matrix
     * @param result Inverted matrix
     * @param n Matrix size
     * @return true if successful, false if matrix is singular
     */
    static bool invertMatrix(const float* matrix, float* result, int n);

    /**
     * @brief Copy matrix content
     * @param source Source matrix
     * @param dest Destination matrix
     * @param size Number of elements to copy
     */
    static void matrixCopy(const float* source, float* dest, int size);

    /**
     * @brief Set matrix to zero
     * @param matrix Matrix to clear
     * @param size Number of elements
     */
    static void matrixClear(float* matrix, int size);

    /**
     * @brief Set matrix to identity
     * @param matrix Matrix to set as identity
     * @param n Matrix dimension (n x n)
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
};

#endif