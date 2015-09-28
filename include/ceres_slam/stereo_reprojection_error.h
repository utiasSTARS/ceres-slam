#ifndef CERES_SLAM_STEREO_REPROJECTION_ERROR_
#define CERES_SLAM_STEREO_REPROJECTION_ERROR_

#include <ceres/ceres.h>

#include <ceres_slam/stereo_camera.h>

namespace ceres_slam {

class StereoReprojectionError : public ceres::SizedCostFunction<
                                            4,  // Residual dimension
                                            3,  // Current vehicle translation
                                            3,  // Current vehicle rotation
                                            3>  // Current map point position
{
public:
    StereoReprojectionError(StereoCamera::ConstPtr& camera,
                            StereoCamera::Observation observation,
                            StereoCamera::ObservationVariance variance);

    virtual ~StereoReprojectionError();

    //! Evaluates the reprojection error and jacobians for Ceres.
    //! parameters[0][0-2] is the camera translation in the global frame
    //! parameters[1][0-2] is the axis-angle camera rotation in the global frame
    //! parametsrs[2][0-2] is the map point in the global frame
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(
                            const StereoCamera::ConstPtr& camera,
                            const StereoCamera::Observation observation,
                            const StereoCamera::ObservationVariance variance);

private:
    StereoCamera::ConstPtr camera_;
    StereoCamera::Observation observation_;
    StereoCamera::ObservationVariance variance_;

}; // class StereoReprojectionError

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_REPROJECTION_ERROR_
