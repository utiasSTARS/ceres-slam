#ifndef CERES_SLAM_NORMAL_ERROR_H_
#define CERES_SLAM_NORMAL_ERROR_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

//! Normal error cost function for Ceres with automatic Jacobians
class NormalErrorAutomatic {
public:
    //! Vector type
    typedef Vector3D<double> Vector;

    //! Constructor with fixed model parameters
    NormalErrorAutomatic(const Vector& obs_normal_c,
                         const Vector::Covariance& stiffness) :
        obs_normal_c_(obs_normal_c),
        stiffness_(stiffness) { }

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const T_c_g_ceres,
                    const T* const normal_g_ceres,
                    T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef Vector3D<T> VectorT;
        typedef Eigen::Matrix<T, 3, 1> ResidualVectorT;

        // Camera pose in the global frame
        Eigen::Map<const SE3T> T_c_g(T_c_g_ceres);

        // Normal vector at the map point
        Eigen::Map<const VectorT> normal_g(normal_g_ceres); // Global frame
        VectorT normal_c = T_c_g * normal_g;                // Camera frame

        // Compute the residuals
        Eigen::Map<ResidualVectorT> residuals(residuals_ceres);
        residuals = stiffness_.cast<T>()
                    * (normal_c - obs_normal_c_.cast<T>());

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const Vector& obs,
                                       const Vector::Covariance& stiffness) {
        return( new ceres::AutoDiffCostFunction
                    <NormalErrorAutomatic,
                       3,  // Residual dimension
                       12, // Compact SE(3) vehicle pose (3 trans + 9 rot)
                       3>  // Map point normal
                       (new NormalErrorAutomatic(obs, stiffness) )
              );
    }

private:
    //! Observed normal vector in the camera frame
    Vector obs_normal_c_;
    //! Normal vector stiffness matrix (inverse sqrt of covariance matrix)
    Vector::Covariance stiffness_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_NORMAL_ERROR_H_
