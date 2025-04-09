/**
 * @file utils.hpp
 * @brief custom utilities header for this project
 */






#ifndef UTILS_HPP_
#define UTILS_HPP_

// VERBOSITY
#ifndef VERBOSE
#define VERBOSE 1
#endif

#if VERBOSE==0
#define LOG_INFO(M)
#else
#define LOG_INFO(M) RCLCPP_INFO(this->get_logger(),M); // log message
#endif



#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <stdexcept>
#include <cuda_runtime.h>
#include <cublas_v2.h>

namespace Utils{
    tf2::Quaternion getQuaternion(double, double, double);
    std::vector<std::vector<double>> matrixMul2D(const std::vector<std::vector<double>>*, const std::vector<std::vector<double>>*, const cublasHandle_t*, cublasOperation_t transpose_A=CUBLAS_OP_N, cublasOperation_t transpose_B=CUBLAS_OP_N);
    std::vector<std::vector<double>> matrixAdd2D(const std::vector<std::vector<double>> *, const std::vector<std::vector<double>> *, const cublasHandle_t*);
    double euclideanDistance(const std::vector<double>*, const std::vector<double>*);
    std::vector<int> NNAssociate(const std::vector<std::vector<double>>*, const std::vector<std::vector<double>>*, const double*);
    std::vector<std::vector<double>> transpose2D(const std::vector<std::vector<double>>*A,const cublasHandle_t*);
}



#endif