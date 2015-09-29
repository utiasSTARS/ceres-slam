#include <ceres_slam/stereo_reprojection_error.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/operators.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace ceres_slam {

StereoReprojectionError::StereoReprojectionError(
                                StereoCamera::ConstPtr& camera,
                                StereoCamera::Observation observation,
                                StereoCamera::ObservationVariance variance) :
    camera_(camera),
    observation_(observation),
    variance_(variance) {}

StereoReprojectionError::~StereoReprojectionError() {}


bool StereoReprojectionError::Evaluate(double const* const* parameters,
                                       double* residuals,
                                       double** jacobians) const {
    // Check if we need to compute jacobians
    bool need_jacobians = jacobians != nullptr && jacobians[0] != nullptr;

    // Camera translation in the global frame
    Eigen::Map<const Eigen::Vector3d> r_g_c_g(&parameters[0][0]);

    // Camera rotation in the global frame.
    // NOTE: Ceres assumes row major matrix storage
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> C_c_g;
    ceres::AngleAxisToRotationMatrix<double>(&parameters[1][0], C_c_g.data());

    // Put camera pose into a transformation matrix for convenience
    Eigen::Affine3d T_c_g;
    T_c_g.linear() = C_c_g;
    T_c_g.translation() = -C_c_g * r_g_c_g;

    // Map point in the global frame
    Eigen::Map<const Eigen::Vector3d> r_g_f_g(&parameters[2][0]);

    // Measurement residuals
    Eigen::Map<Eigen::Vector4d> residuals_eigen(&residuals[0]);

    // Transform map point into the camera frame
    Eigen::Vector3d r_c_f_c = T_c_g * r_g_f_g;

    // Project into stereo camera, computing jacobian if needed
    StereoCamera::Observation predicted_observation;
    StereoCamera::ObservationJacobian camera_jacobian;

    if(need_jacobians) {
        predicted_observation =
            camera_->pointToObservation(r_c_f_c, &camera_jacobian);
    }
    else {
        predicted_observation = camera_->pointToObservation(r_c_f_c);
    }

    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4>>
        es(variance_.asDiagonal());
    Eigen::Matrix<double, 4, 4> stiffness = es.operatorInverseSqrt();

    // Compute the residuals
    // NOTE: Updating residuals_eigen will also update residuals
    residuals_eigen = stiffness * (observation_ - predicted_observation);

    // Compute jacobians if needed
    if(need_jacobians) {
        Eigen::Matrix<double, 4, 3, Eigen::RowMajor> premult =
            stiffness * (-camera_jacobian);

        // Jacobian of residuals w.r.t. camera translation
        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>>
            translation_jacobian(&jacobians[0][0]);
        translation_jacobian = -premult * C_c_g;

        // Jacobian of residuals w.r.t. camera rotation
        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>>
            rotation_jacobian(&jacobians[1][0]);
        rotation_jacobian = premult * operatorWedge<double>(r_c_f_c);

        // Jacobian of residuals w.r.t. map point position
        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>>
            point_jacobian(&jacobians[2][0]);
        point_jacobian = premult * C_c_g;
    }

    return true;
}

ceres::CostFunction* StereoReprojectionError::Create(
                        const StereoCamera::ConstPtr& camera,
                        const StereoCamera::Observation observation,
                        const StereoCamera::ObservationVariance variance) {
    return (new StereoReprojectionError(camera, observation, variance));
}

} // namespace ceres_slam
