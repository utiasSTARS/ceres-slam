#ifndef CERES_SLAM_STEREO_CAMERA_H_
#define CERES_SLAM_STEREO_CAMERA_H_

#include <sensor_msgs/CameraInfo.h>

#include <memory>
#include <Eigen/Core>

namespace ceres_slam {

class StereoCamera {
public:
    // Typedefs
    typedef std::shared_ptr<StereoCamera> Ptr;
    typedef const std::shared_ptr<StereoCamera> ConstPtr;

    typedef Eigen::Vector4d Observation;
    typedef Eigen::Vector3d Point3D;
    typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> ProjectionMatrix;

    // Constructors, destructor
    StereoCamera(const sensor_msgs::CameraInfo &left_camera_info, const sensor_msgs::CameraInfo &right_camera_info);
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

    // Stereo functions
    Observation pointToObservation(const Point3D &pt_c);
    Point3D observationToPoint(const Observation &obs);

private:
    ProjectionMatrix P_left_, P_right_;

}; // class StereoCamera

} // namespace ceres_slam

#endif // CERES_SLAM_STEREO_CAMERA_H_
