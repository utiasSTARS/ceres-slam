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
    bool need_jacobians = jacobians_ceres != nullptr
                          && jacobians_ceres[0] != nullptr;

    // Camera pose in global frame
    Eigen::Map<const SE3::TangentVector> xi_c_g(&parameters_ceres[0][0]);
    SE3 T_c_g = SE3::exp(xi_c_g);

    // Map point in the global frame
    Point pt_g(&parameters_ceres[1][0]);

    // Map the measurement residuals to an Eigen vector for convenience
    Eigen::Map<ResidualVector> residuals(&residuals_ceres[0]);

    // Transform map point into the camera frame
    Point pt_c = T_c_g * pt_c;

    // Project into stereo camera, computing jacobian if needed
    Camera::Observation predicted_observation;
    Camera::ObservationJacobian camera_jacobian;

    if(need_jacobians) {
        predicted_observation =
            camera_->project(pt_c, &camera_jacobian);
    }
    else {
        predicted_observation = camera_->project(pt_c);
    }

    // Compute the residuals
    // NOTE: Updating residuals_eigen will also update residuals
    residuals = stiffness_ * (observation_ - predicted_observation);

    // Compute jacobians if needed
    if(need_jacobians) {
        // d(residual) / d(pt_c)
        // (3x3)(3x3) ==> (3x3)
        Camera::ObservationJacobian factor = -stiffness_ * camera_jacobian;

        // d(pt_c) / d(T_c_g)
        // (3x3)(3x6) ==> (3x6)
        Eigen::Map<SE3::TransformedPointJacobian>
            pose_jacobian(&jacobians_ceres[0][0]);
        pose_jacobian = factor * SE3::odot(pt_c);

        // d(pt_c) / d(pt_g)
        // (3x3)(3x3) ==> (3x3)
        Eigen::Map<Camera::ObservationJacobian>
            point_jacobian(&jacobians_ceres[1][0]);
        point_jacobian = factor * T_c_g.rotation().matrix();
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
    return ( new ceres::AutoDiffCostFunction <StereoReprojectionErrorAutomatic,
                                                3,  // Residual dimension
                                                6,  // Vehicle pose vector
                                                3>  // Map point position
                    (new StereoReprojectionErrorAutomatic(
                            camera, observation, stiffness))
            );
}

} // namespace ceres_slam
