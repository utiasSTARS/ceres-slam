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
    static const int dim = 3;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Point Jacobian type
    typedef Eigen::Matrix<Scalar, 3, dim> PointJacobian;
    //! Observation type
    typedef Eigen::Matrix<Scalar, dim, 1> Observation;
    //! Observation variance type
    typedef Eigen::Matrix<Scalar, dim, 1> ObservationVariance;
    //! Observation Jacobian type
    typedef Eigen::Matrix<Scalar, dim, 3> ObservationJacobian;

    //! Copy constructor
    StereoCamera( StereoCamera& other ) :
        _fu(other.fu()), _fv(other.fv()),
        _cu(other.cu()), _cv(other.cv()),
        _b(other.b()) { }
    //! Construct from parameters
    StereoCamera( Scalar fu, Scalar fv, Scalar cu, Scalar cv, Scalar b ) :
        _fu(fu), _fv(fv),
        _cu(cu), _cv(cv),
        _b(b) { }
    //! Construct from ROS CameraInfo messages
    StereoCamera(const sensor_msgs::CameraInfoConstPtr& left_camera_info,
                 const sensor_msgs::CameraInfoConstPtr& right_camera_info) :
        _fu(left_camera_info->P[0]), _fv(left_camera_info->P[5]),
        _cu(left_camera_info->P[2]), _cv(left_camera_info->P[6]),
        _b(-right_camera_info->P[3] / right_camera_info->P[0]) { }

    //! Return horizontal focal length
    const Scalar fu() const { return _fu; }
    //! Return vertical focal length
    const Scalar fv() const { return _fv; }
    //! Return horizontal principal point coordinate
    const Scalar cu() const { return _cu; }
    //! Return vertical horizontal principal point coordinate
    const Scalar cv() const { return _cv; }
    //! Return baseline
    const Scalar b() const { return _b; }

    //! Projects a 3D point in the camera frame into the camera
    //! to get a 3D stereo observation.
    const Observation pointToObservation(
        const Point& pt_c, ObservationJacobian* jacobian_ptr = nullptr) const {
        Observation obs; // [u_l, v_l, d]
        obs(0) = fu() * pt_c(0) / pt_c(2) + cu();
        obs(1) = fv() * pt_c(1) / pt_c(2) + cv();
        obs(2) = fu() * b() / pt_c(2);

        if(jacobian_ptr != nullptr) {
            ObservationJacobian jacobian = *jacobian_ptr;

            double one_over_z = 1 / pt_c(2);
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
    //! from a 3D stereo observation.
    const Point observationToPoint(
        const Observation& obs, PointJacobian* jacobian_ptr = nullptr) const {
        Point pt_c;
        pt_c(2) = obs(2) / (fu() * b());
        pt_c(1) = pt_c(2) * (obs(1) - cv()) / fv();
        pt_c(0) = pt_c(2) * (obs(0) - cu()) / fu();

        if(jacobian_ptr != nullptr) {
            std::cerr << "StereoCamera::observationToPoint jacobian not yet implemented." << std::endl;
        }

        return pt_c;
    }

    //! Ostream operator for StereoCamera
    friend std::ostream& operator<<(
        std::ostream& os, const StereoCamera<Scalar>& c ) {
        os << "StereoCamera" << std::endl
           << "fu: " << c.fu() << std::endl
           << "fv: " << c.fv() << std::endl
           << "cu: " << c.cu() << std::endl
           << "cv: " << c.cv() << std::endl
           << "b: "  << c.b()  << std::endl;
        return os;
    }

private:
    Scalar _fu; //!< Horizontal focal length [px]
    Scalar _fv; //!< Vertical focal length [px]
    Scalar _cu; //!< Horizontal principal point coordinate [px]
    Scalar _cv; //!< Vertical principal point coordinate [px]
    Scalar _b;  //!< Stereo baseline [m]
}; // class StereoCamera

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_CAMERA_H_
