#include <Eigen/Core>

namespace ceres_slam {

//! Wedge operator as defined by Barfoot (a.k.a. cross product matrix)
template <typename T>
Eigen::Matrix<T, 3, 3> operatorWedge(const Eigen::Matrix<T, 3, 1> vec) {
    Eigen::Matrix<T, 3, 3> mat;
    mat << T(0.0), -vec(2), vec(1),
           vec(2), T(0.0), -vec(0),
           -vec(1), vec(0), T(0.0);

    return mat;
}

//! Inverse wedge operator as defined by Barfoot
template <typename T>
Eigen::Matrix<T, 3, 1> operatorInverseWedge(const Eigen::Matrix<T, 3, 3> mat) {
    Eigen::Matrix<T, 3, 1> vec;
    vec << mat(2,1), mat(0,2), mat(1,0);

    return vec;
}

} // namespace ceres_slam
