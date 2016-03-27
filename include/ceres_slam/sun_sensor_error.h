#ifndef CERES_SLAM_SUN_SENSOR_ERROR_H_
#define CERES_SLAM_SUN_SENSOR_ERROR_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

//! Sun sensor error cost function for Ceres with automatic Jacobians
class SunSensorErrorAutomatic {
   public:
    //! Vector type
    typedef Vector3D<double> Vector;

    //! Constructor
    SunSensorErrorAutomatic(const Vector& observed_sun_dir_c,
                            const Vector& expected_sun_dir_g,
                            const Vector::Covariance& stiffness)
        : observed_sun_dir_c_(observed_sun_dir_c),
          expected_sun_dir_g_(expected_sun_dir_g),
          stiffness_(stiffness) {
        observed_sun_dir_c_.normalize();
        expected_sun_dir_g_.normalize();
    }

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const T_c_g_ceres, T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef Vector3D<T> VectorT;
        typedef Eigen::Matrix<T, 3, 1> ResidualVectorT;

        // Camera pose in the global frame
        Eigen::Map<const SE3T> T_c_g(T_c_g_ceres);

        // Expected sun direction in the camera frame
        VectorT expected_sun_dir_g = expected_sun_dir_g_.cast<T>();
        VectorT expected_sun_dir_c = T_c_g * expected_sun_dir_g;

        // Do the casting here for convenience
        VectorT observed_sun_dir_c = observed_sun_dir_c_.cast<T>();

        // Compute the residual
        Eigen::Map<ResidualVectorT> residuals(residuals_ceres);

        if (T(1) - expected_sun_dir_c.dot(observed_sun_dir_c) < T(0.1)) {
            residuals = stiffness_.cast<T>() *
                        (expected_sun_dir_c - observed_sun_dir_c);
        } else {
            residuals = ResidualVectorT::Zero();
        }

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const Vector& observed_sun_dir_c,
                                       const Vector& expected_sun_dir_g,
                                       const Vector::Covariance& stiffness) {
        return (new ceres::AutoDiffCostFunction<SunSensorErrorAutomatic,
                                                3,   // Residual dimension
                                                12>  // Compact SE(3) vehicle
                                                     // pose (3 trans + 9 rot)
                (new SunSensorErrorAutomatic(observed_sun_dir_c,
                                             expected_sun_dir_g, stiffness)));
    }

   private:
    //! Sun direction observation in the camera frame
    Vector observed_sun_dir_c_;
    //! Expected sun direction in the global frame
    Vector expected_sun_dir_g_;
    //! Observation stiffness matrix (inverse sqrt of covariance matrix)
    Vector::Covariance stiffness_;
};  // class SunSensorErrorAutomatic

}  // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_SUN_SENSOR_ERROR_H_ */
