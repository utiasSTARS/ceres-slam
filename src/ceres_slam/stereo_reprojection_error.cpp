#include <ceres_slam/stereo_reprojection_error.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/geometry.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>

namespace ceres_slam {

StereoReprojectionErrorAnalytic::StereoReprojectionErrorAnalytic(
                            Camera::ConstPtr camera,
                            const Camera::Observation& observation,
                            const Camera::ObservationCovariance& stiffness) :
    camera_(camera),
    observation_(observation),
    stiffness_(stiffness) { }

StereoReprojectionErrorAnalytic::~StereoReprojectionErrorAnalytic() { }


bool StereoReprojectionErrorAnalytic::Evaluate(double const* const* parameters,
                                               double* residuals,
                                               double** jacobians) const {
    // Check if we need to compute jacobians
    bool need_jacobians = jacobians != nullptr && jacobians[0] != nullptr;

    // Camera pose in global frame
    Eigen::Map<const SE3::TangentVector> xi_c_g(&parameters[0][0]);
    SE3 T_c_g = SE3::exp(xi_c_g);

    // Map point in the global frame
    Point r_g_f_g(&parameters[1][0]);

    // Map the measurement residuals to an Eigen vector for convenience
    Eigen::Map<Eigen::Vector3d> residuals_eigen(&residuals[0]);

    // Transform map point into the camera frame
    Point r_c_f_c = T_c_g * r_g_f_g;

    // Project into stereo camera, computing jacobian if needed
    Camera::Observation predicted_observation;
    Camera::ObservationJacobian camera_jacobian;

    if(need_jacobians) {
        predicted_observation =
            camera_->project(r_c_f_c, &camera_jacobian);
    }
    else {
        predicted_observation = camera_->project(r_c_f_c);
    }

    // Compute the residuals
    // NOTE: Updating residuals_eigen will also update residuals
    residuals_eigen = stiffness_ * (observation_ - predicted_observation);

    // Compute jacobians if needed
    if(need_jacobians) {
        // Common factor to both Jacobians
        // (3x3)(3x3) ==> (3x3)
        Camera::ObservationJacobian factor = stiffness_ * camera_jacobian;

        // Jacobian of residuals w.r.t. camera pose
        // (3x3)(3x6) ==> (3x6)
        Eigen::Map<SE3::TransformedPointJacobian>
            pose_jacobian(&jacobians[0][0]);
        pose_jacobian = factor * SE3::odot(r_c_f_c);

        // Jacobian of residuals w.r.t. map point position
        // (3x3)(3x3) ==> (3x3)
        Eigen::Map<Camera::ObservationJacobian>
            point_jacobian(&jacobians[1][0]);
        point_jacobian = factor * T_c_g.rotation().matrix();

        // std::cout << "stiffness" << std::endl
        //           << stiffness_ << std::endl;
        // std::cout << "camera_jacobian" << std::endl
        //           << camera_jacobian << std::endl;
        // std::cout << "residuals" << std::endl
        //           << residuals_eigen << std::endl;
        // std::cout << "pose_jacobian" << std::endl
        //           << pose_jacobian << std::endl;
        // std::cout << "point_jacobian" << std::endl
        //           << point_jacobian << std::endl;
    }

    return true;
}

ceres::CostFunction* StereoReprojectionErrorAnalytic::Create(
                        const Camera::ConstPtr camera,
                        const Camera::Observation& observation,
                        const Camera::ObservationCovariance& stiffness) {
    return ( new StereoReprojectionErrorAnalytic(
                    camera, observation, stiffness) );
}


StereoReprojectionErrorAutomatic::StereoReprojectionErrorAutomatic(
                            Camera::ConstPtr camera,
                            const Camera::Observation& observation,
                            const Camera::ObservationCovariance& stiffness) :
    camera_(camera),
    observation_(observation),
    stiffness_(stiffness) { }


ceres::CostFunction* StereoReprojectionErrorAutomatic::Create(
                        const Camera::ConstPtr camera,
                        const Camera::Observation& observation,
                        const Camera::ObservationCovariance& stiffness) {
    return ( new ceres::AutoDiffCostFunction
                <StereoReprojectionErrorAutomatic,
                    3,  // Residual dimension
                    6,  // Vehicle pose vector
                    3>  // Map point position
                (new StereoReprojectionErrorAutomatic(
                    camera, observation, stiffness)) );
}

} // namespace ceres_slam
