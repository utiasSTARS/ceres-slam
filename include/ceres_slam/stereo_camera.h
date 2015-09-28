#ifndef CERES_SLAM_STEREO_CAMERA_H_
#define CERES_SLAM_STEREO_CAMERA_H_

#include <sensor_msgs/CameraInfo.h>

#include <memory>
#include <Eigen/Core>

namespace ceres_slam {

//! A ROS-compatible Stereo camera model
class StereoCamera {
public:
    typedef std::shared_ptr<StereoCamera> Ptr;
    typedef const std::shared_ptr<StereoCamera> ConstPtr;

    typedef Eigen::Vector3d Point;
    typedef Eigen::Vector3d PointVariance;
    typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> PointJacobian;
    typedef Eigen::Vector4d Observation;
    typedef Eigen::Vector4d ObservationVariance;
    typedef Eigen::Matrix<double, 4, 3, Eigen::RowMajor> ObservationJacobian;
    typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> ProjectionMatrix;

    StereoCamera(const sensor_msgs::CameraInfo& left_camera_info, const sensor_msgs::CameraInfo& right_camera_info);
    ~StereoCamera();

    // Accessors
    const ProjectionMatrix P_left();
    const ProjectionMatrix P_right();
    const double fu_left();
    const double fv_left();
    const double cu_left();
    const double cv_left();
    const double Tu_left();
    const double Tv_left();
    const double fu_right();
    const double fv_right();
    const double cu_right();
    const double cv_right();
    const double Tu_right();
    const double Tv_right();
    const double b();

    //! Projects a 3D point in the camera frame into the camera
    //! to get a 4D stereo observation.
    Observation pointToObservation(const Point& pt_c, ObservationJacobian* jacobian_ptr = nullptr);

    //! Triangulates a 3D point in the camera frame
    //! from a 4D stereo observation.
    Point observationToPoint(const Observation& obs, PointJacobian* jacobian_ptr = nullptr);

private:
    ProjectionMatrix P_left_;   //!< Left camera 3x4 projection matrix
    ProjectionMatrix P_right_;  //!< Right camera 3x4 projection matrix

}; // class StereoCamera

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_CAMERA_H_
