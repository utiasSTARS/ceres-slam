#include <iostream>
#include <cstdlib>

#include <sensor_msgs/CameraInfo.h>

#include <ceres_slam/stereo_camera.h>

int main() {
    ceres_slam::StereoCamera::Point pt;
    pt << 1.0, 1.0, 1.0;

    double fu = 300.0;
    double fv = 300.0;
    double cu = 300.0;
    double cv = 300.0;
    double b = 0.25;
    double Tu_left = -b*fu;

    sensor_msgs::CameraInfo left_camera_info, right_camera_info;

    Eigen::Map<ceres_slam::StereoCamera::ProjectionMatrix> P_left(left_camera_info.P.data());
    P_left << fu, 0, cu, 0,
              0, fv, cv, 0,
              0, 0, 1, 0;

    Eigen::Map<ceres_slam::StereoCamera::ProjectionMatrix> P_right(right_camera_info.P.data());
    P_right = P_left;
    P_right(0,3) = Tu_left;

    ceres_slam::StereoCamera cam(left_camera_info, right_camera_info);
    // ceres_slam::StereoCamera::Ptr cam(new ceres_slam::StereoCamera(left_camera_info, right_camera_info));
    // ceres_slam::StereoCamera::ConstPtr cam(new ceres_slam::StereoCamera(left_camera_info, right_camera_info));

    std::cout << "cam.P_left() = " << std::endl << cam.P_left() << std::endl;
    std::cout << "cam.P_right() = " << std::endl << cam.P_right() << std::endl;

    ceres_slam::StereoCamera::Observation obs = cam.pointToObservation(pt);
    std::cout << "pointToObservation: " << std::endl << obs << std::endl;

    ceres_slam::StereoCamera::Point pt2 = cam.observationToPoint(obs);
    std::cout << "observationToPoint: " << std::endl << pt2 << std::endl;

    return EXIT_SUCCESS;
}
