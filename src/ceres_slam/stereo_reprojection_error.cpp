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


bool StereoReprojectionErrorAnalytic::Evaluate(
    double const* const* parameters_ceres,
    double* residuals_ceres,
    double** jacobians_ceres) const {
    // Local typedefs for convenience
    typedef Eigen::Matrix<double, 3, 1> ResidualVector;

    // Check if we need to compute jacobians
    bool need_pose_jacobian = jacobians_ceres != nullptr
                            && jacobians_ceres[0] != nullptr;
    bool need_point_jacobian = jacobians_ceres != nullptr
                            && jacobians_ceres[1] != nullptr;

    // Camera pose in global frame
    Eigen::Map<const SE3::TangentVector> xi_c_g(parameters_ceres[0]);
    SE3 T_c_g = SE3::exp(xi_c_g);

    // Map point in the global frame
    Point pt_g(parameters_ceres[1]);

    // Transform map point into the camera frame, computing jacobian if needed
    Point pt_c;
    SE3::TransformedPointJacobian transformed_point_jacobian;

    if(need_pose_jacobian) {
        pt_c = T_c_g.transform(pt_g, &transformed_point_jacobian);
    }
    else {
        pt_c = T_c_g * pt_g;
    }

    // Project into stereo camera, computing jacobian if needed
    Camera::Observation predicted_observation;
    Camera::ProjectionJacobian camera_jacobian;

    if(need_pose_jacobian || need_point_jacobian) {
        predicted_observation =
            camera_->project(pt_c, &camera_jacobian);
        // d(residual) / d(pt_c)
        // (3x3)(3x3) ==> (3x3)
        camera_jacobian = stiffness_ * camera_jacobian;
    }
    else {
        predicted_observation = camera_->project(pt_c);
    }

    // Compute the residuals
    Eigen::Map<ResidualVector> residuals(residuals_ceres);
    residuals = stiffness_ * (predicted_observation - observation_);

    // Compute jacobians if needed
    if(need_pose_jacobian) {
        // d(pt_c) / d(T_c_g)
        // (3x3)(3x6) ==> (3x6)
        Eigen::Map<SE3::TransformedPointJacobian>
            pose_jacobian(jacobians_ceres[0]);
        pose_jacobian = camera_jacobian * transformed_point_jacobian;
    }

    if(need_point_jacobian) {
        // d(pt_c) / d(pt_g)
        // (3x3)(3x3) ==> (3x3)
        Eigen::Map<Camera::ProjectionJacobian>
            point_jacobian(jacobians_ceres[1]);
        point_jacobian = camera_jacobian * T_c_g.rotation().matrix();
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

} // namespace ceres_slam
