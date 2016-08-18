#ifndef CERES_SLAM_RELATIVE_POSE_ERROR_HPP_
#define CERES_SLAM_RELATIVE_POSE_ERROR_HPP_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.hpp>

namespace ceres_slam {

//! Relative pose error cost function for Ceres with automatic Jacobians
class RelativePoseErrorAutomatic {
   public:
    //! SE(3) type
    typedef SE3Group<double> SE3;

    //! Constructor
    RelativePoseErrorAutomatic(const SE3& T_2_1_ref,
                               const SE3::AdjointMatrix& stiffness)
        : T_2_1_ref_(T_2_1_ref), stiffness_(stiffness){};

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const T_1_0_ceres, const T* const T_2_0_ceres,
                    T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef typename SE3T::TangentVector TangentVectorT;

        // The error between two SE(3) poses A and B is the vector from
        // A to B in the tangent space of A (or vice versa)
        Eigen::Map<const SE3T> T_1_0(T_1_0_ceres);
        Eigen::Map<const SE3T> T_2_0(T_2_0_ceres);
        Eigen::Map<TangentVectorT> residuals(residuals_ceres);

        SE3T T_1_2_est = T_1_0 * T_2_0.inverse();
        SE3T T_residual = T_2_1_ref_.cast<T>() * T_1_2_est;
        residuals = stiffness_.cast<T>() * SE3T::log(T_residual);

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const SE3& T_2_1_ref,
                                       const SE3::AdjointMatrix& stiffness) {
        return (new ceres::AutoDiffCostFunction<RelativePoseErrorAutomatic,
                                                6,   // Residual dimension
                                                12,  // 2x Compact SE(3) pose
                                                12>  // (3 trans + 9 rot)
                (new RelativePoseErrorAutomatic(T_2_1_ref, stiffness)));
    }

   private:
    //! Relative transformation measurement from F1 to F2
    SE3 T_2_1_ref_;
    //! 6x6 pose stiffness matrix (inverse sqrt of covariance matrix)
    SE3::AdjointMatrix stiffness_;

};  // class RelativePoseErrorAutomatic

}  // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_RELATIVE_POSE_ERROR_HPP_ */
