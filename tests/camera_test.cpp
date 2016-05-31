#include <cstdlib>
#include <iostream>

#include <ceres_slam/stereo_camera.h>

// TODO: Rewrite this as a proper unit test

using StereoCamera = ceres_slam::StereoCamera<double>;

int main() {
    double fu = 100.0;
    double fv = 200.0;
    double cu = 300.0;
    double cv = 400.0;
    double b = 0.25;

    StereoCamera cam(fu, fv, cu, cv, b);
    std::cout << cam << std::endl;

    StereoCamera::Point pt;
    pt << 1.0, 2.0, 3.0;
    std::cout << pt << std::endl;

    StereoCamera::ProjectionJacobian obs_jac;
    StereoCamera::Observation obs = cam.project(pt, &obs_jac);
    std::cout << "project: " << obs.transpose() << std::endl;
    std::cout << obs_jac << std::endl;

    StereoCamera::TriangulationJacobian pt2_jac;
    StereoCamera::Point pt2 = cam.triangulate(obs, &pt2_jac);
    std::cout << "triangulate: " << pt2 << std::endl;
    std::cout << pt2_jac << std::endl;

    return EXIT_SUCCESS;
}
