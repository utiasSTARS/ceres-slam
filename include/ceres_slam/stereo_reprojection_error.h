#ifndef CERES_SLAM_STEREO_REPROJECTION_ERROR_H_
#define CERES_SLAM_STEREO_REPROJECTION_ERROR_H_

#include <ceres/ceres.h>

#include <ceres_slam/stereo_camera.h>

namespace ceres_slam {

//! Stereo reprojection error cost function for Ceres with analytic Jacobians
class StereoReprojectionErrorAnalytic : public ceres::SizedCostFunction<
                                            3,  // Residual dimension
                                            6,  // Vehicle pose vector
                                            3>  // Map point position
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
    StereoReprojectionErrorAnalytic(
        Camera::ConstPtr camera,
        const Camera::Observation& observation,
        const Camera::ObservationCovariance& stiffness);

    //! Destructor
    virtual ~StereoReprojectionErrorAnalytic();

    //! Evaluates the reprojection error and jacobians for Ceres.
    /*! parameters[0][0-2] is the vehicle translation in the global frame
        parameters[0][3-5] is the vehicle rotation in the global frame
        parametsrs[1][0-2] is the map point in the global frame
    */
    virtual bool Evaluate(double const* const* parameters_ceres,
                          double* residuals_ceres,
                          double** jacobians_ceres) const;

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(
                            const Camera::ConstPtr camera,
                            const Camera::Observation& observation,
                            const Camera::ObservationCovariance& stiffness);

private:
    //! Camera model
    Camera::ConstPtr camera_;
    //! Stereo observation
    Camera::Observation observation_;
    //! Observation stiffness matrix (inverse sqrt of covariance matrix)
    Camera::ObservationCovariance stiffness_;

}; // class StereoReprojectionErrorAnalytic

//! Stereo reprojection error cost function for Ceres with automatic Jacobians
class StereoReprojectionErrorAutomatic {
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
    StereoReprojectionErrorAutomatic(
        Camera::ConstPtr camera,
        const Camera::Observation& observation,
        const Camera::ObservationCovariance& stiffness);

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const xi_c_g_ceres,
                    const T* const pt_g_ceres,
                    T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef typename SE3T::TangentVector TangentVector;
        typedef Point3D<T> PointT;
        typedef Eigen::Matrix<T, 3, 1> ResidualVectorT;
        typedef StereoCamera<T> CameraT;
        typedef typename CameraT::Observation Observation;

        // Camera pose in global frame
        Eigen::Map<const TangentVector> xi_c_g(xi_c_g_ceres);
        SE3T T_c_g = SE3T::exp(xi_c_g);

        // Map point in the global frame
        PointT pt_g(pt_g_ceres);

        // Map the measurement residuals to an Eigen vector for convenience
        Eigen::Map<ResidualVectorT> residuals(residuals_ceres);

        // Transform map point into the camera frame
        PointT pt_c = T_c_g * pt_g;

        // Project into stereo camera, computing jacobian if needed
        CameraT cameraT(T(camera_->fu()), T(camera_->fv()),
                        T(camera_->cu()), T(camera_->cv()),
                        T(camera_->b()));
        Observation predicted_observation;

        predicted_observation = cameraT.project(pt_c);

        // Compute the residuals
        // NOTE: Updating residuals_eigen will also update residuals
        residuals = stiffness_.cast<T>()
            * (observation_.cast<T>() - predicted_observation);

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(
                            const Camera::ConstPtr camera,
                            const Camera::Observation& observation,
                            const Camera::ObservationCovariance& stiffness);

private:
    //! Camera model
    Camera::ConstPtr camera_;
    //! Stereo observation
    Camera::Observation observation_;
    //! Observation stiffness matrix (inverse sqrt of covariance matrix)
    Camera::ObservationCovariance stiffness_;

}; // class StereoReprojectionErrorAutomatic

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_REPROJECTION_ERROR_H_
