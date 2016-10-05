#ifndef CERES_SLAM_SO3_LOCAL_PARAM_H_
#define CERES_SLAM_SO3_LOCAL_PARAM_H_

#include <ceres/ceres.h>

#include "../liegroups/so3group.hpp"

namespace ceres_slam {

//! SO(3) group perturbation via so(3) algebra
class SO3LocalParameterization {
   public:
    template <typename T>
    bool operator()(const T *C_op_ceres, const T *psi_ceres,
                    T *C_op_new_ceres) const {
        // Local typedefs for convenience
        typedef SO3Group<T> SO3T;
        typedef typename SO3T::TangentVector TangentVectorT;

        // Initial unperturbed transformation
        Eigen::Map<const SO3T> C_op(C_op_ceres);
        // The perturbation in the transformation's tangent space
        Eigen::Map<const TangentVectorT> psi(psi_ceres);
        // Final perturbed transformation
        Eigen::Map<SO3T> C_op_new(C_op_new_ceres);

        // Perturb the transformation
        C_op_new = SO3T::exp(psi) * C_op;

        return true;
    }

    //! Factory to hide the construction of the LocalParameterization object
    //! from the client code.
    static ceres::LocalParameterization *Create() {
        return (new ceres::AutoDiffLocalParameterization<
                SO3LocalParameterization,
                9,  // SO(3) matrix elements
                3>  // so(3) tangent vector
                );
    }
};  // class SO3LocalParameterization

}  // namespace ceres_slam

#endif  // CERES_SLAM_SO3_LOCAL_PARAM_H_
