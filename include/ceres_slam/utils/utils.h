#ifndef CERES_SLAM_UTILS_H_
#define CERES_SLAM_UTILS_H_

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ceres_slam {

//! Templated version of fmax for use with ceres
template <typename Scalar>
Scalar fmax(Scalar a, Scalar b) {
    return ((a >= b) ? a : b);
}

//! Templated version of fmin for use with ceres
template <typename Scalar>
Scalar fmin(Scalar a, Scalar b) {
    return ((a <= b) ? a : b);
}

//! String formatting for Eigen file IO
const Eigen::IOFormat CommaInitFmt(4, 1, ",", ",", "", "", "", "");

//! Print an array given its address and size
template <typename Scalar>
void print_array(const Scalar* array, const int n) {
    for (int i = 0; i < n; ++i) std::cout << *(array + i) << std::endl;
}

//! Split a delimited string into a vector of tokens
std::vector<std::string> split(const std::string str, const char del);

}  // namespace ceres_slam

#endif  // CERES_SLAM_UTILS_H_
