#include <ceres_slam/stereo_camera.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/CameraInfo.h>

namespace ceres_slam {

StereoCamera::StereoCamera(const sensor_msgs::CameraInfo& left_camera_info, const sensor_msgs::CameraInfo& right_camera_info) {
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

StereoCamera::Observation StereoCamera::pointToObservation(const StereoCamera::Point& pt_c, ObservationJacobian* jacobian_ptr) {
    Point projected_left = P_left() * pt_c.homogeneous();
    Point projected_right = P_right() * pt_c.homogeneous();

    Observation obs; // [u_l, v_l, u_r, v_r]
    obs(0) = projected_left(0) / projected_left(2);
    obs(1) = projected_left(1) / projected_left(2);
    obs(2) = projected_right(0) / projected_right(2);
    obs(3) = projected_right(1) / projected_right(2);

    if(jacobian_ptr != nullptr) {
        ObservationJacobian jacobian = *jacobian_ptr;

        double one_over_z = 1 / pt_c(2);
        double one_over_z2 = one_over_z * one_over_z;

        // d(u_l) / d(pt_c)
        jacobian(0,0) = fu_left() * one_over_z;
        jacobian(0,1) = 0.0;
        jacobian(0,2) = -(fu_left() * pt_c(0) + Tu_left()) * one_over_z2;

        // d(v_l) / d(pt_c)
        jacobian(1,0) = 0.0;
        jacobian(1,1) = fv_left() * one_over_z;
        jacobian(1,2) = -(fv_left() * pt_c(1) + Tv_left()) * one_over_z2;

        // d(u_r) / d(pt_c)
        jacobian(2,0) = fu_right() * one_over_z;
        jacobian(2,1) = 0.0;
        jacobian(2,2) = -(fu_right() * pt_c(0) + Tu_right()) * one_over_z2;

        // d(v_r) / d(pt_c)
        jacobian(3,0) = 0.0;
        jacobian(3,1) = fv_right() * one_over_z;
        jacobian(3,2) = -(fv_right() * pt_c(1) + Tv_right()) * one_over_z2;
    }

    return obs;
}

StereoCamera::Point StereoCamera::observationToPoint(const StereoCamera::Observation& obs, PointJacobian* jacobian_ptr) {
    Point pt_c;
    pt_c(2) = Tu_right() * fu_left() /
        ( (obs(2) - cu_right()) * fu_left()
         -(obs(0) - cu_left()) * fu_right() );
    pt_c(1) = pt_c(2) * (obs(1) - cv_left()) / fv_left();
    pt_c(0) = pt_c(2) * (obs(0) - cu_left()) / fu_left();

    if(jacobian_ptr != nullptr) {
        std::cerr << "StereoCamera::observationToPoint jacobian not yet implemented." << std::endl;
    }

    return pt_c;
}

} // namespace ceres_slam
