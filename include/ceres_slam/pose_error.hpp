#ifndef CERES_SLAM_POSE_ERROR_HPP_
#define CERES_SLAM_POSE_ERROR_HPP_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.hpp>

namespace ceres_slam {

//! Pose error cost function for Ceres with automatic Jacobians
class PoseErrorAutomatic {
public:
    //! SE(3) type
    typedef SE3Group<double> SE3;

    //! Constructor
    PoseErrorAutomatic(const SE3& T_k_0_ref) : T_k_0_ref_(T_k_0_ref) { };

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const T_k_0_ceres, T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef typename SE3T::TangentVector TangentVectorT;

        // The error between two SE(3) poses A and B is the vector from
        // A to B in the tangent space of A (or vice versa)
        Eigen::Map<const SE3T> T_k_0(T_k_0_ceres);
        Eigen::Map<TangentVectorT> residuals(residuals_ceres);

        SE3T T_residual = T_k_0.inverse() * T_k_0_ref_.cast<T>();
        residuals = stiffness_.cast<T>() * SE3T::log(T_residual);

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const SE3& T_k_0_ref) {
        return (new ceres::AutoDiffCostFunction<PoseErrorAutomatic,
                                                6,  // Residual dimension
                                                12> // Compact SE(3) pose
                                                    // (3 trans + 9 rot)
                (new PoseErrorAutomatic(T_k_0_ref)));
    }

private:
    //! Reference pose as transformation from frame 0 to frame K
    SE3 T_k_0_ref_;
    //! 6x6 pose stiffness matrix (inverse sqrt of covariance matrix)
    SE3::AdjointMatrix stiffness_;

}; // class PoseErrorAutomatic

} // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_POSE_ERROR_HPP_ */
