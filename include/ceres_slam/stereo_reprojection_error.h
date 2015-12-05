#ifndef CERES_SLAM_STEREO_REPROJECTION_ERROR_H_
#define CERES_SLAM_STEREO_REPROJECTION_ERROR_H_

#include <ceres/ceres.h>

#include <ceres_slam/stereo_camera.h>

namespace ceres_slam {

class StereoReprojectionError : public ceres::SizedCostFunction<
                                            3,  // Residual dimension
                                            6,  // Current vehicle pose
                                            3>  // Current map point position
{
public:
    //! Camera type
    typedef StereoCamera<double> Camera;
    //! SO(3) type
    typedef SO3Group<double> SO3;
    //! SE(3) type
    typedef SE3Group<double> SE3;
    //! Point type
    typedef Point3D<double> Point;
    //! Vector type
    typedef Vector3D<double> Vector;

    //! Constructor with fixed model parameters
    StereoReprojectionError(Camera::ConstPtr& camera,
                            Camera::Observation observation,
                            Camera::ObservationVariance variance);

    //! Destructor
    virtual ~StereoReprojectionError();

    //! Evaluates the reprojection error and jacobians for Ceres.
    //! parameters[0][0-2] is the vehicle translation in the global frame
    //! parameters[0][3-5] is the vehicle rotation in the global frame
    //! parametsrs[1][0-2] is the map point in the global frame
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(
                            const Camera::ConstPtr& camera,
                            const Camera::Observation observation,
                            const Camera::ObservationVariance variance);

private:
    Camera::ConstPtr camera_;                 //!< Camera model
    Camera::Observation observation_;         //!< Stereo observation
    Camera::ObservationVariance variance_;    //!< Observation variance

}; // class StereoReprojectionError

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_REPROJECTION_ERROR_H_
