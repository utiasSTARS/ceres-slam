#include <ceres_slam/stereo_camera.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/CameraInfo.h>

namespace ceres_slam {

StereoCamera::StereoCamera(const sensor_msgs::CameraInfo &left_camera_info, const sensor_msgs::CameraInfo &right_camera_info) {
    P_left_ = Eigen::Map<const ProjectionMatrix>(left_camera_info.P.data());
    P_right_ = Eigen::Map<const ProjectionMatrix>(right_camera_info.P.data());
}

StereoCamera::~StereoCamera() {}

const StereoCamera::ProjectionMatrix StereoCamera::P_left() { return P_left_; }
const StereoCamera::ProjectionMatrix StereoCamera::P_right() { return P_right_; }
const double StereoCamera::fu_left() { return P_left_(0,0); }
const double StereoCamera::fv_left() { return P_left_(1,1); }
const double StereoCamera::cu_left() { return P_left_(0,2); }
const double StereoCamera::cv_left() { return P_left_(1,2); }
const double StereoCamera::Tu_left() { return P_left_(0,3); }
const double StereoCamera::Tv_left() { return P_left_(1,3); }
const double StereoCamera::fu_right() { return P_right_(0,0); }
const double StereoCamera::fv_right() { return P_right_(1,1); }
const double StereoCamera::cu_right() { return P_right_(0,2); }
const double StereoCamera::cv_right() { return P_right_(1,2); }
const double StereoCamera::Tu_right() { return P_right_(0,3); }
const double StereoCamera::Tv_right() { return P_right_(1,3); }
const double StereoCamera::b() { return -P_right_(0,3) / fu_right(); }

StereoCamera::Observation StereoCamera::pointToObservation(const StereoCamera::Point3D &pt_c) {
    Point3D projected_left = P_left() * pt_c.homogeneous();
    Point3D projected_right = P_right() * pt_c.homogeneous();

    Observation obs; // [u_l, v_l, u_r, v_r]
    obs(0) = projected_left(0) / projected_left(2);
    obs(1) = projected_left(1) / projected_left(2);
    obs(2) = projected_right(0) / projected_right(2);
    obs(3) = projected_right(1) / projected_right(2);

    return obs;
}

StereoCamera::Point3D StereoCamera::observationToPoint(const StereoCamera::Observation &obs) {
    Point3D pt_c;
    pt_c(2) = Tu_right() * fu_left() /
        ( (obs(2) - cu_right()) * fu_left()
         -(obs(0) - cu_left()) * fu_right() );
    pt_c(1) = pt_c(2) * (obs(1) - cv_left()) / fv_left();
    pt_c(0) = pt_c(2) * (obs(0) - cu_left()) / fu_left();

    return pt_c;
}

} // namespace ceres_slam
