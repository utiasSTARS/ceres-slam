#include <iostream>
#include <cstdlib>

#include <sensor_msgs/CameraInfo.h>

#include <ceres_slam/stereo_camera.h>

using StereoCamera = ceres_slam::StereoCamera<double>;

int main() {
    StereoCamera::Point pt(1.0, 1.0, 1.0);

    double fu = 100.0;
    double fv = 200.0;
    double cu = 300.0;
    double cv = 400.0;
    double b = 0.25;
    double Tu_left = -b*fu;

    sensor_msgs::CameraInfoPtr left_camera_info(new sensor_msgs::CameraInfo);
    sensor_msgs::CameraInfoPtr right_camera_info(new sensor_msgs::CameraInfo);

    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>
        P_left(left_camera_info->P.data());
    P_left << fu, 0, cu, 0,
              0, fv, cv, 0,
              0, 0, 1, 0;

    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>
        P_right(right_camera_info->P.data());
    P_right = P_left;
    P_right(0,3) = Tu_left;

    StereoCamera cam(left_camera_info, right_camera_info);

    std::cout << cam << std::endl;

    StereoCamera::Observation obs = cam.pointToObservation(pt);
    std::cout << "pointToObservation: " << std::endl << obs << std::endl;

    StereoCamera::Point pt2 = cam.observationToPoint(obs);
    std::cout << "observationToPoint: " << std::endl << pt2 << std::endl;

    return EXIT_SUCCESS;
}
