#ifndef CERES_SLAM_STEREO_REPROJECTION_ERROR_H_
#define CERES_SLAM_STEREO_REPROJECTION_ERROR_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/stereo_camera.hpp>

namespace ceres_slam {

//! Stereo reprojection error cost function for Ceres with automatic Jacobians
class StereoReprojectionErrorAutomatic {
   public:
    //! Camera type
    typedef StereoCamera<double> Camera;

    //! Constructor with fixed model parameters
    StereoReprojectionErrorAutomatic(
        const Camera::ConstPtr camera, const Camera::Observation& observation,
        const Camera::ObservationCovariance& stiffness)
        : camera_(camera), observation_(observation), stiffness_(stiffness) {
        kitti_weight =
            0.05 / (fabs(observation(0) - camera->cu()) / camera->cu() + 0.05);
    }

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const T_c_g_ceres, const T* const pt_g_ceres,
                    T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef Point3D<T> PointT;
        typedef Eigen::Matrix<T, 3, 1> ResidualVectorT;
        typedef StereoCamera<T> CameraT;
        typedef typename CameraT::Observation ObservationT;

        // Camera pose in the global frame
        Eigen::Map<const SE3T> T_c_g(T_c_g_ceres);

        // Map point
        Eigen::Map<const PointT> pt_g(pt_g_ceres);  // Global frame
        PointT pt_c = T_c_g * pt_g;                 // Camera frame

        // Project into stereo camera, computing jacobian if needed
        ObservationT predicted_observation = camera_->cast<T>().project(pt_c);

        // Compute the residuals
        Eigen::Map<ResidualVectorT> residuals(residuals_ceres);
        residuals = stiffness_.cast<T>() *
                    (predicted_observation - observation_.cast<T>());

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(
        const Camera::ConstPtr camera, const Camera::Observation& observation,
        const Camera::ObservationCovariance& stiffness) {
        return (new ceres::AutoDiffCostFunction<
                StereoReprojectionErrorAutomatic,
                3,   // Residual dimension
                12,  // Compact SE(3) vehicle pose (3 trans + 9 rot)
                3>   // Map point position
                (new StereoReprojectionErrorAutomatic(camera, observation,
                                                      stiffness)));
    }

   private:
    //! Camera model
    Camera::ConstPtr camera_;
    //! Stereo observation
    Camera::Observation observation_;
    //! Observation stiffness matrix (inverse sqrt of covariance matrix)
    Camera::ObservationCovariance stiffness_;
    //! KITTI bias correction weight
    double kitti_weight;

};  // class StereoReprojectionErrorAutomatic

}  // namespace ceres_slam

#endif  // CERES_SLAM_STEREO_REPROJECTION_ERROR_H_
