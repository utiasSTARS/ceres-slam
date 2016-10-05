#ifndef CERES_SLAM_POSE_COST_HPP_
#define CERES_SLAM_POSE_COST_HPP_

#include <ceres/ceres.h>

#include "../liegroups/se3group.hpp"

namespace ceres_slam {

//! Pose error cost function for Ceres with automatic Jacobians
class PoseCostAutomatic {
   public:
    //! SE(3) type
    typedef SE3Group<double> SE3;

    //! Constructor
    PoseCostAutomatic(const SE3& T_k_0_ref, const SE3::AdjointMatrix& stiffness)
        : T_k_0_ref_(T_k_0_ref), stiffness_(stiffness){};

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

        SE3T T_residual = T_k_0_ref_.cast<T>() * T_k_0.inverse();

        // std::cout << "T_residual:\n";
        // for (uint i = 0; i < 12; ++i) {
        //     std::cout << T_residual.data()[i] << "\n";
        // }

        residuals = SE3T::log(T_residual);

        // std::cout << "\nresiduals:\n";
        // for (uint i = 0; i < 6; ++i) {
        //     std::cout << residuals_ceres[i] << std::endl;
        // }

        residuals = stiffness_.cast<T>() * residuals;

        // std::cout << "\nstiffness * residuals:\n";
        // for (uint i = 0; i < 6; ++i) {
        //     std::cout << residuals_ceres[i] << std::endl;
        // }

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const SE3& T_k_0_ref,
                                       const SE3::AdjointMatrix& stiffness) {
        return (new ceres::AutoDiffCostFunction<PoseCostAutomatic,
                                                6,   // Residual dimension
                                                12>  // Compact SE(3) pose
                                                     // (3 trans + 9 rot)
                (new PoseCostAutomatic(T_k_0_ref, stiffness)));
    }

   private:
    //! Reference pose as transformation from frame k to frame 0
    SE3 T_k_0_ref_;
    //! 6x6 pose stiffness matrix (inverse sqrt of covariance matrix)
    SE3::AdjointMatrix stiffness_;

};  // class PoseCostAutomatic

}  // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_POSE_COST_HPP_ */
