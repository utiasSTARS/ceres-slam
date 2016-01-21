#ifndef CERES_SLAM_PERTURBATIONS_H_
#define CERES_SLAM_PERTURBATIONS_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

class SE3Perturbation {
public:
    template <typename T>
    bool operator()(const T* T_op_ceres,
                    const T* epsilon_ceres,
                    T* T_op_new_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef typename SE3T::TangentVector TangentVectorT;

        // Initial unperturbed transformation
        Eigen::Map<const SE3T> T_op(T_op_ceres);
        // The perturbation in the transformation's tangent space
        Eigen::Map<const TangentVectorT> epsilon(epsilon_ceres);
        // Final perturbed transformation
        Eigen::Map<SE3T> T_op_new(T_op_new_ceres);

        // Perturb the transformation
        T_op_new = SE3T::exp(epsilon) * T_op;

        return true;
    }

    //! Factort to hide the construction of the LocalParameterization object
    //! from the client code.
    static ceres::LocalParameterization* Create() {
        return(new ceres::AutoDiffLocalParameterization
                <SE3Perturbation,
                    12, // Compact SE(3) (3 trans + 9 rot)
                    6> // se(3) tangent vector
              );
    }
}; // class SE3Perturbation

}

#endif // CERES_SLAM_PERTURBATIONS_H_