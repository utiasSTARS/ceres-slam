#ifndef CERES_SLAM_STEREO_CAMERA_H_
#define CERES_SLAM_STEREO_CAMERA_H_

#include <sensor_msgs/CameraInfo.h>

#include <memory>
#include <Eigen/Core>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

//! A ROS-compatible Stereo camera model
template <typename Scalar>
class StereoCamera {
public:
    //! Pointer type
    typedef std::shared_ptr<StereoCamera> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<StereoCamera> ConstPtr;
    //! Dimension of the observation (u, v, d)
    static const int obs_dim = 3;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Point Jacobian type
    typedef Eigen::Matrix<Scalar, Point::dim, obs_dim, Eigen::RowMajor>
        PointJacobian;
    //! Observation type
    typedef Eigen::Matrix<Scalar, obs_dim, 1> Observation;
    //! Observation variance type
    typedef Eigen::Matrix<Scalar, obs_dim, 1>
        ObservationVariance;
    //! Observation covariance matrix type
    typedef Eigen::Matrix<Scalar, obs_dim, obs_dim, Eigen::RowMajor>
        ObservationCovariance;
    //! Observation Jacobian type
    typedef Eigen::Matrix<Scalar, obs_dim, Point::dim, Eigen::RowMajor>
        ObservationJacobian;

    //! Copy constructor
    StereoCamera( const StereoCamera& other ) :
        fu_(other.fu()), fv_(other.fv()),
        cu_(other.cu()), cv_(other.cv()),
        b_(other.b()) { }
    //! Construct from parameters
    StereoCamera( const Scalar fu, const Scalar fv,
                  const Scalar cu, const Scalar cv,
                  const Scalar b ) :
        fu_(fu), fv_(fv),
        cu_(cu), cv_(cv),
        b_(b) { }
    //! Construct from ROS CameraInfo messages
    StereoCamera(const sensor_msgs::CameraInfoConstPtr& left_camera_info,
                 const sensor_msgs::CameraInfoConstPtr& right_camera_info) :
        fu_(left_camera_info->P[0]), fv_(left_camera_info->P[5]),
        cu_(left_camera_info->P[2]), cv_(left_camera_info->P[6]),
        b_(-right_camera_info->P[3] / right_camera_info->P[0]) { }

    //! Return horizontal focal length
    const Scalar fu() const { return fu_; }
    //! Return vertical focal length
    const Scalar fv() const { return fv_; }
    //! Return horizontal principal point coordinate
    const Scalar cu() const { return cu_; }
    //! Return vertical horizontal principal point coordinate
    const Scalar cv() const { return cv_; }
    //! Return baseline
    const Scalar b() const { return b_; }

    //! Projects a 3D point in the camera frame into the camera
    //! to get a uvd stereo observation.
    const Observation project(
        const Point& pt_c, ObservationJacobian* jacobian_ptr = nullptr) const {
        double one_over_z = 1. / pt_c(2);

        Observation obs; // [u_l, v_l, d]
        obs(0) = fu() * pt_c(0) * one_over_z + cu();
        obs(1) = fv() * pt_c(1) * one_over_z + cv();
        obs(2) = fu() * b() * one_over_z;

        if(jacobian_ptr != nullptr) {
            ObservationJacobian& jacobian = *jacobian_ptr;

            double one_over_z2 = one_over_z * one_over_z;

            // d(u_l) / d(pt_c)
            jacobian(0,0) = fu() * one_over_z;
            jacobian(0,1) = 0.0;
            jacobian(0,2) = -fu() * pt_c(0) * one_over_z2;

            // d(v_l) / d(pt_c)
            jacobian(1,0) = 0.0;
            jacobian(1,1) = fv() * one_over_z;
            jacobian(1,2) = -fv() * pt_c(1) * one_over_z2;

            // d(d) / d(pt_c)
            jacobian(2,0) = fu() * one_over_z;
            jacobian(2,1) = 0.0;
            jacobian(2,2) = -fu() * b() * one_over_z2;
        }

        return obs;
    }

    //! Triangulates a 3D point in the camera frame
    //! from a uvd stereo observation.
    const Point triangulate(
        const Observation& obs, PointJacobian* jacobian_ptr = nullptr) const {
        Point pt_c;
        pt_c(2) = fu() * b() / obs(2);
        pt_c(1) = pt_c(2) * (obs(1) - cv()) / fv();
        pt_c(0) = pt_c(2) * (obs(0) - cu()) / fu();

        if(jacobian_ptr != nullptr) {
            std::cerr << "StereoCamera::triangulate jacobian not yet implemented." << std::endl;
        }

        return pt_c;
    }

    //! Ostream operator for StereoCamera
    friend std::ostream& operator<<( std::ostream& os, const
                                     StereoCamera<Scalar>& c ) {
        os << "StereoCamera" << std::endl
           << "fu: " << c.fu() << std::endl
           << "fv: " << c.fv() << std::endl
           << "cu: " << c.cu() << std::endl
           << "cv: " << c.cv() << std::endl
           << "b: "  << c.b();
        return os;
    }

private:
    Scalar fu_; //!< Horizontal focal length [px]
    Scalar fv_; //!< Vertical focal length [px]
    Scalar cu_; //!< Horizontal principal point coordinate [px]
    Scalar cv_; //!< Vertical principal point coordinate [px]
    Scalar b_;  //!< Stereo baseline [m]
}; // class StereoCamera

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_CAMERA_H_
