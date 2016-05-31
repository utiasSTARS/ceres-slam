#ifndef CERES_SLAM_PERTURBATIONS_H_
#define CERES_SLAM_PERTURBATIONS_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.h>

namespace ceres_slam {

//! SO(3) group perturbation via so(3) algebra
class SO3Perturbation {
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
                SO3Perturbation,
                9,  // SO(3) matrix elements
                3>  // so(3) tangent vector
                );
    }
};  // class SO3Perturbation

//! SE(3) group perturbation via se(3) algebra
class SE3Perturbation {
   public:
    template <typename T>
    bool operator()(const T *T_op_ceres, const T *epsilon_ceres,
                    T *T_op_new_ceres) const {
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

    //! Factory to hide the construction of the LocalParameterization object
    //! from the client code.
    static ceres::LocalParameterization *Create() {
        return (new ceres::AutoDiffLocalParameterization<
                SE3Perturbation,
                12,  // Compact SE(3) (3 trans + 9 rot)
                6>   // se(3) tangent vector
                );
    }
};  // class SE3Perturbation

//! Orthogonal perturbation of unit vector
/*!
    Project delta onto the plane whose normal is x
    and apply only the component of delta that is orthogonal
    to x. This is done by subtracting the component of delta
    that is along the normal direction:

    \f$ \delta_\perp = \delta - \frac{\delta \cdot x}{||x||^2} x \f$
*/
class UnitVectorPerturbation {
   public:
    template <typename T>
    bool operator()(const T *x_ceres, const T *delta_ceres,
                    T *x_plus_delta_ceres) const {
        typedef Vector3D<T> VectorT;

        Eigen::Map<const VectorT> x(x_ceres);
        Eigen::Map<const VectorT> delta(delta_ceres);
        Eigen::Map<VectorT> x_plus_delta(x_plus_delta_ceres);

        VectorT delta_orthogonal = delta - (delta.dot(x) / x.squaredNorm()) * x;

        x_plus_delta = x + delta_orthogonal;

        x_plus_delta.normalize();

        return true;
    }

    //! Factory to hide the construction of the LocalParameterization object
    //! from the client code.
    static ceres::LocalParameterization *Create() {
        return (new ceres::AutoDiffLocalParameterization<UnitVectorPerturbation,
                                                         3, 3>);
    }
};  // class UnitVectorPerturbation

}  // namespace ceres_slam

#endif  // CERES_SLAM_PERTURBATIONS_H_
