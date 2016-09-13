#ifndef CERES_SLAM_SUN_SENSOR_ERROR_2D_HPP_
#define CERES_SLAM_SUN_SENSOR_ERROR_2D_HPP_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/utils/utils.hpp>

namespace ceres_slam {

//! Sun sensor error cost function for Ceres with automatic Jacobians
class SunSensorError2DAutomatic {
   public:
    //! Vector type
    typedef Vector3D<double> Vector;

    //! Constructor
    SunSensorError2DAutomatic(const Vector& observed_sun_dir_c,
                              const Vector& expected_sun_dir_g,
                              const double stiffness,
                              const double cosine_dist_thresh)
        : observed_sun_dir_c_(observed_sun_dir_c),
          expected_sun_dir_g_(expected_sun_dir_g),
          stiffness_(stiffness) {
        observed_sun_dir_c_.normalize();
        expected_sun_dir_g_.normalize();
        az_thresh_ = acos(1. - cosine_dist_thresh);
    }

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const T_c_g_ceres, T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef Vector3D<T> VectorT;

        // Camera pose in the global frame
        Eigen::Map<const SE3T> T_c_g(T_c_g_ceres);

        // Expected sun direction in the camera frame
        VectorT expected_sun_dir_g = expected_sun_dir_g_.cast<T>();
        VectorT expected_sun_dir_c = T_c_g * expected_sun_dir_g;

        // Do the casting here for convenience
        VectorT observed_sun_dir_c = observed_sun_dir_c_.cast<T>();

        // Convert to azimuth and zenith
        // T expected_zen = acos(-expected_sun_dir_c(1));
        T expected_az = atan2(expected_sun_dir_c(0), expected_sun_dir_c(2));

        // T observed_zen = acos(-observed_sun_dir_c(1));
        T observed_az = atan2(observed_sun_dir_c(0), observed_sun_dir_c(2));

        T residual_az = expected_az - observed_az;
        // T residual_zen = expected_zen - observed_zen;

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

        // Threshold the azimuth error to reject outliers
        if (-T(az_thresh_) < residual_az && residual_az < T(az_thresh_)) {
            residuals_ceres[0] = T(stiffness_) * residual_az;
        } else {
            residuals_ceres[0] = T(0);
        }

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const Vector& observed_sun_dir_c,
                                       const Vector& expected_sun_dir_g,
                                       const double stiffness,
                                       const double cosine_dist_thresh) {
        return (new ceres::AutoDiffCostFunction<SunSensorError2DAutomatic,
                                                1,   // Residual dimension
                                                12>  // Compact SE(3) vehicle
                                                     // pose (3 trans + 9 rot)
                (new SunSensorError2DAutomatic(observed_sun_dir_c,
                                               expected_sun_dir_g, stiffness,
                                               cosine_dist_thresh)));
    }

   private:
    //! Sun direction observation in the camera frame
    Vector observed_sun_dir_c_;
    //! Expected sun direction in the global frame
    Vector expected_sun_dir_g_;
    //! Observation stiffness matrix (inverse sqrt of covariance matrix)
    double stiffness_;
    //! Azimuth threshold for outlier rejection
    double az_thresh_;
};  // class SunSensorErrorAutomatic

}  // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_SUN_SENSOR_ERROR_2D_HPP_ */
