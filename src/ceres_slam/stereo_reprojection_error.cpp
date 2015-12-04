#include <ceres_slam/stereo_reprojection_error.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/geometry.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace ceres_slam {

StereoReprojectionError::StereoReprojectionError(
                                Camera::ConstPtr& camera,
                                Camera::Observation observation,
                                Camera::ObservationVariance variance) :
    camera_(camera),
    observation_(observation),
    variance_(variance) { }

StereoReprojectionError::~StereoReprojectionError() { }


bool StereoReprojectionError::Evaluate(double const* const* parameters,
                                       double* residuals,
                                       double** jacobians) const {
    // Check if we need to compute jacobians
    bool need_jacobians = jacobians != nullptr && jacobians[0] != nullptr;

    // Camera pose in global frame
    SE3 T_c_g = SE3::exp(&parameters[0][0]);

    // Map point in the global frame
    Point r_g_f_g(&parameters[1][0]);

    // Measurement residuals
    Eigen::Map<Eigen::Vector4d> residuals_eigen(&residuals[0]);

    // Transform map point into the camera frame
    Point r_c_f_c = T_c_g * r_g_f_g;

    // Project into stereo camera, computing jacobian if needed
    Camera::Observation predicted_observation;
    Camera::ObservationJacobian camera_jacobian;

    if(need_jacobians) {
        predicted_observation =
            camera_->pointToObservation(r_c_f_c, &camera_jacobian);
    }
    else {
        predicted_observation = camera_->pointToObservation(r_c_f_c);
    }

    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Camera::ObservationCovariance>
        es(variance_.asDiagonal());
    Camera::ObservationCovariance stiffness = es.operatorInverseSqrt();

    // Compute the residuals
    // NOTE: Updating residuals_eigen will also update residuals
    residuals_eigen = stiffness * (observation_ - predicted_observation);

    // Compute jacobians if needed
    if(need_jacobians) {
        Camera::ObservationJacobian factor = stiffness * camera_jacobian;

        // Jacobian of residuals w.r.t. camera pose
        Eigen::Map<SE3::TransformedPointJacobian>
            pose_jacobian(&jacobians[0][0]);
        pose_jacobian = factor
            * SE3::transformed_point_jacobian(r_c_f_c);

        // Jacobian of residuals w.r.t. map point position
        Eigen::Map<Camera::ObservationJacobian>
            point_jacobian(&jacobians[1][0]);
        point_jacobian = factor * T_c_g.rotation().matrix();
    }

    return true;
}

ceres::CostFunction* StereoReprojectionError::Create(
                        const Camera::ConstPtr& camera,
                        const Camera::Observation observation,
                        const Camera::ObservationVariance variance) {
    return (new StereoReprojectionError(camera, observation, variance));
}

} // namespace ceres_slam
