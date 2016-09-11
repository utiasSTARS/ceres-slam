#ifndef CERES_SLAM_SUN_SENSOR_ERROR_HPP_
#define CERES_SLAM_SUN_SENSOR_ERROR_HPP_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/utils/utils.hpp>

namespace ceres_slam {

//! Sun sensor error cost function for Ceres with automatic Jacobians
class SunSensorErrorAutomatic {
   public:
    //! Vector type
    typedef Vector3D<double> Vector;
    //! Residual covariance type
    typedef Eigen::Matrix<double, 2, 2, Eigen::RowMajor> ResidualCovariance;

    //! Constructor
    SunSensorErrorAutomatic(const Vector& observed_sun_dir_c,
                            const Vector& expected_sun_dir_g,
                            const ResidualCovariance& stiffness)
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
        typedef Eigen::Matrix<T, 2, 1> ResidualVectorT;

        // Camera pose in the global frame
        Eigen::Map<const SE3T> T_c_g(T_c_g_ceres);

        // Rotate the frames to avoid singularities where most of the data are
        typename SE3Group<double>::TransformationMatrix T_c2_c_matrix;
        T_c2_c_matrix << 1., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0.,
            0., 0., 1.;
        SE3T T_c2_c(T_c2_c_matrix.cast<T>());

        // Expected sun direction in the camera frame
        VectorT expected_sun_dir_g = expected_sun_dir_g_.cast<T>();
        VectorT expected_sun_dir_c = T_c2_c * T_c_g * expected_sun_dir_g;

        // Do the casting here for convenience
        VectorT observed_sun_dir_c = observed_sun_dir_c_.cast<T>();
        observed_sun_dir_c = T_c2_c * observed_sun_dir_c;

        // Convert to azimuth and zenith
        T expected_az = acos(-expected_sun_dir_c(1));
        T expected_zen = atan2(expected_sun_dir_c(0), expected_sun_dir_c(2));

        T observed_az = acos(-observed_sun_dir_c(1));
        T observed_zen = atan2(observed_sun_dir_c(0), observed_sun_dir_c(2));

        // Compute the residual
        Eigen::Map<ResidualVectorT> residuals(residuals_ceres);

        // Threshold the cosine distance to reject outliers
        if (T(1) - expected_sun_dir_c.dot(observed_sun_dir_c) < T(0.3)) {
            T residual_az = expected_az - observed_az;
            T residual_zen = expected_zen - observed_zen;
            // T residual_zen = T(0);

            // Correct azimuth wraparound by choosing the smallest angular
            // distance.
            // Note that atan2 returns values in the range [-pi,pi],
            // so we don't need to do a modulo 2*pi on the error since
            // abs(error) is at most 2*pi
            if (residual_az > T(pi)) {
                residual_az = residual_az - T(2 * pi);
            } else if (residual_az < -T(pi)) {
                residual_az = residual_az + T(2 * pi);
            }

            residuals << residual_az, residual_zen;
            residuals = stiffness_.cast<T>() * residuals;
        } else {
            residuals = ResidualVectorT::Zero();
        }

        // std::cout << "residuals:\n";
        // for (uint i = 0; i < 2; ++i) {
        //     std::cout << residuals[i] << "\n";
        // }

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const Vector& observed_sun_dir_c,
                                       const Vector& expected_sun_dir_g,
                                       const ResidualCovariance& stiffness) {
        return (new ceres::AutoDiffCostFunction<SunSensorErrorAutomatic,
                                                2,   // Residual dimension
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
    ResidualCovariance stiffness_;
};  // class SunSensorErrorAutomatic

}  // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_SUN_SENSOR_ERROR_HPP_ */
