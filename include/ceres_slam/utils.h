#ifndef CERES_SLAM_UTILS_H_
#define CERES_SLAM_UTILS_H_

#include <Eigen/Core>

namespace ceres_slam {

//! Templated version of fmax for use with ceres
template <typename Scalar>
Scalar fmax(Scalar a, Scalar b) { return ( (a >= b) ? a : b ); }

//! Templated version of fmin for use with ceres
template <typename Scalar>
Scalar fmin(Scalar a, Scalar b) { return ( (a <= b) ? a : b ); }

//! Templated version of abs for use with ceres
template <typename Scalar>
Scalar abs(Scalar a) { return ( (a >= Scalar(0)) ? a : -a ); }

//! String formatting for Eigen file IO
const Eigen::IOFormat CommaInitFmt(4, 1, ",", ",", "", "", "", "");

} // namespace ceres_slam

#endif // CERES_SLAM_UTILS_H_
