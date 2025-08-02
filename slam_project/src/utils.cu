/**
* @file utils.hpp
* @brief custom utilities for this project
*/




#include "slam_project/utils.hpp"
#include <cmath>

 
 
 /**
  * @namespace Utils
  * @brief converts rpy to quaternions
  * @param roll
  * @param pitch
  * @param yaw
  * @return quaternion object of type tf2::Quaternion
  */
tf2::Quaternion Utils::getQuaternion(double roll, double pitch, double yaw){
     tf2::Quaternion q;
     q.setRPY(roll, pitch, yaw);
     return q;
}


/**
 * @brief matrix multiplication of 2D arrays, on GPU
 * @param A AxB (mxn)
 * @param B matrix A (mxk)
 * @param handle reference to CublasHandle_t object
 * @param transpose_A 1 if A should be transposed, 0 if not
 * @param transpose_B 1 if B should be transposed, 0 if not
 * @return resulting array, vector form
 */
std::vector<std::vector<double>> Utils::matrixMul2D(const std::vector<std::vector<double>>*A,const std::vector<std::vector<double>>*B, const cublasHandle_t*handle, cublasOperation_t transpose_A, cublasOperation_t transpose_B){
     if(A->at(0).size() != B->size()){
          throw std::invalid_argument("Inner dimensions of matrix multiplication do not match !");
     }
     int m=A->size(), k=A->at(0).size(), n=B->at(0).size();
     double *c_A, *c_B, *c_C, *d_A, *d_B, *d_C; // CPU memory pointers and GPU memory pointers
     int size_A = m*k*sizeof(double), size_B = k*n*sizeof(double), size_C = m*n*sizeof(double);
     std::vector<std::vector<double>> C(m, std::vector<double>(n, 0.0));

     // Initialising device array variable pointers
     c_A = new double[m*k];
     c_B = new double[k*n];
     c_C = new double[m*n];
     // Copying from vector to device pointer array
     for (int i = 0; i < m; i++){
          for (int j = 0; j < k; j++){
               c_A[i*k+j] = A->at(i).at(j);
          }   
     }
     for (int i = 0; i < k; i++){
          for (int j = 0; j < n; j++){
               c_B[i*n+j] = B->at(i).at(j);
          }   
     }
     
     // Allocate memory on GPU
     cudaMalloc((void**)&d_A, size_A);
     cudaMalloc((void**)&d_B, size_B);
     cudaMalloc((void**)&d_C, size_C);
     // Copy memory from cpu to gpu
     cudaMemcpy(d_A, c_A, size_A, cudaMemcpyHostToDevice);
     cudaMemcpy(d_B, c_B, size_B, cudaMemcpyHostToDevice);
     // Scalling factors
     const double alpha = 1.0f, beta = 0.0f;
     // Actual multiplication !
     cublasDgemm(*handle, transpose_A, transpose_B, m, n, k, &alpha, d_A, m ,d_B, k, &beta, d_C, m);
     // Copy results to cpu
     cudaMemcpy(c_C, d_C, size_C, cudaMemcpyDeviceToHost);
     cudaFree(d_A);
     cudaFree(d_B);
     cudaFree(d_C);
     for (int i=0;i<m;i++){
          for (int j = 0; j < n; j++)
          {
               C[i][j] = c_C[j*m + i];
          }
     }
     delete[] c_A;
     delete[] c_B;
     delete[] c_C;
     return C;
}


/**
 * @brief returns addition of 2 2D matrices, on GPU
 * @param A matrix A
 * @param B matrix B
 * @param handle reference to cublasHandle_t object
 * @returns resulting array, vector form
 */
std::vector<std::vector<double>> Utils::matrixAdd2D(const std::vector<std::vector<double>>*A, const std::vector<std::vector<double>>*B, const cublasHandle_t *handle){
     if(A->size()!=B->size() || A->at(0).size() != B->at(0).size()){
          throw std::invalid_argument("Matrix dimensions don't match for addition operation");
     }
     double *c_A, *c_B, *d_A, *d_B;
     int rows = A->size(), cols=A->at(0).size();
     c_A = new double[rows*cols];
     c_B = new double[rows*cols];
     int size = rows*cols*sizeof(double);
     std::vector<std::vector<double>> C(rows, std::vector<double>(cols, 0.0));

     
     for (int i = 0; i < rows; i++){
          for (int j = 0; j < cols; j++){
               c_A[i*cols+j] = A->at(i).at(j);
               c_B[i*cols+j] = B->at(i).at(j);
          }   
     }
     // Allocate memory on GPU
     cudaMalloc((void**)&d_A, size);
     cudaMalloc((void**)&d_B, size);
     // Copy memory from cpu to gpu
     cudaMemcpy(d_A, c_A, size, cudaMemcpyHostToDevice);
     cudaMemcpy(d_B, c_B, size, cudaMemcpyHostToDevice);
     // addition
     const double alpha = 1.0;
     cublasDaxpy(*handle, rows*cols, &alpha, d_A, 1, d_B, 1);

     // Copy result back to host
     cudaMemcpy(c_B, d_B, size, cudaMemcpyDeviceToHost);

     for (int i = 0; i < rows; i++){
          for (int j = 0; j < cols; j++){
               C[i][j] = c_B[cols*i+j];
          }     
     }
     cudaFree(d_A);
     cudaFree(d_B);
     delete[] c_A, c_B;
     return C;
}


/**
 * @brief calculates euclidean distance between 2 vectors
 * @param A
 * @param B
 * @returns double, distance between A and B
*/
double Utils::euclideanDistance(const std::vector<double> *A, const std::vector<double> *B){
     if (A->size()!=B->size()){
          throw std::invalid_argument("Vector dimensions do not match for euclidean distance");
     }
     double sq_sum=0;
     for (int i = 0; i < A->size(); i++)
     {
          sq_sum+=(A->at(i) - B->at(i))*(A->at(i) - B->at(i));
     }

     return std::sqrt(sq_sum);
     
}


/**
 * @brief Takes 2 vectors and applys nn association algorithm
 * @param sensor_readings pointer pointing to a 2D vector, of shape nx2 or nx3 (columns are x y z coords), of new sensor_readings
 * @param readings_history pointer pointing to a 2D vector, of shape nx2 or nx3 (columns are x y z coords), of previous readings (history)
 * @param threshold distance threshold to classify a new data point as repeated or unique data point
 * @returns an vector array, of size same as sensor_readings, where value at index k is index of corresponding data point in readings_history to which the value at index k of sensor_readings corresponds to.
 * -1 if the readings corresponds to no data point in the history (Which is outside the threshold limit)
 */
std::vector<int> Utils::NNAssociate(const std::vector<std::vector<double>> *sensor_readings, const std::vector<std::vector<double>> *readings_history, const double *threshold){
     std::vector<int> indices(sensor_readings->size(), 0);
     for(int i=0;i<sensor_readings->size();i++){
          double min_reading=0.0;
          int min_reading_index=0;
          for(int j=0; j<readings_history->size();j++){
               double distance = Utils::euclideanDistance(&sensor_readings->at(i), &readings_history->at(j));
               if(distance<min_reading){
                    min_reading=distance;
                    min_reading_index=j;
               }
          }
          indices.at(i) = (min_reading>*threshold?-1:min_reading_index);
     }

     return indices;
}


/**
 * @brief Computes and returns transpose of a matrix
 */
std::vector<std::vector<double>> Utils::transpose2D(const std::vector<std::vector<double>>* A, const cublasHandle_t* handle) {
     if (!A || A->empty() || A->at(0).empty()) {
         throw std::invalid_argument("Input matrix is empty");
     }
 
     int rows = A->size();
     int cols = A->at(0).size();
 
     // Flatten A into 1D array (row-major)
     std::vector<double> h_A(rows * cols);
     for (int i = 0; i < rows; ++i) {
         if (A->at(i).size() != cols) {
             throw std::invalid_argument("Input matrix rows have inconsistent sizes");
         }
         for (int j = 0; j < cols; ++j) {
             h_A[i * cols + j] = A->at(i)[j];
         }
     }
 
     // Allocate device memory
     double* d_A = nullptr;
     double* d_C = nullptr;
     cudaMalloc(&d_A, rows * cols * sizeof(double));
     cudaMalloc(&d_C, rows * cols * sizeof(double));
 
     // Copy A to device
     cudaMemcpy(d_A, h_A.data(), rows * cols * sizeof(double), cudaMemcpyHostToDevice);
 
     // Set parameters for cuBLAS Dgeam
     const double alpha = 1.0;
     const double beta = 0.0;
 
     // Perform C = alpha * A^T + beta * B^T
     cublasDgeam(
         *handle,
         CUBLAS_OP_T, CUBLAS_OP_N,    // Transpose A, do not transpose B
         rows, cols,                  // Dimensions of C
         &alpha,
         d_A, cols,                   // Leading dimension of A
         &beta,
         d_A, cols,                   // B not really used
         d_C, rows                    // Leading dimension of C (after transpose)
     );
 
     // Copy result back to host
     std::vector<double> h_C(rows * cols);
     cudaMemcpy(h_C.data(), d_C, rows * cols * sizeof(double), cudaMemcpyDeviceToHost);
 
     // Convert back to 2D vector
     std::vector<std::vector<double>> result(cols, std::vector<double>(rows));
     for (int i = 0; i < cols; ++i) {
         for (int j = 0; j < rows; ++j) {
             result[i][j] = h_C[i * rows + j];
         }
     }
 
     // Free device memory
     cudaFree(d_A);
     cudaFree(d_C);
 
     return result;
 }

double Utils::normalizeAngle(double angle){
     angle = std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
     return angle;
}