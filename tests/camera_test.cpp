#include <cstdlib>
#include <iostream>

#include <ceres_slam/stereo_camera.hpp>

// TODO: Rewrite this as a proper unit test

using StereoCamera = ceres_slam::StereoCamera<double>;

int main() {
    double fu = 707.0912;
    double fv = 707.0912;
    double cu = 601.8873;
    double cv = 183.1104;
    double b = 0.535105804;

    StereoCamera cam(fu, fv, cu, cv, b);
    std::cout << cam << std::endl;

    // StereoCamera::Point pt;
    // pt << 1.0, 2.0, 3.0;
    // std::cout << pt << std::endl;

    StereoCamera::Observation obs;
    obs << 60, 71, 12;
    std::cout << obs << std::endl;

    StereoCamera::TriangulationJacobian pt_jac;
    StereoCamera::Point pt = cam.triangulate(obs, &pt_jac);
    std::cout << "triangulate: " << pt << std::endl;
    std::cout << pt_jac << std::endl;

    StereoCamera::ProjectionJacobian obs2_jac;
    StereoCamera::Observation obs2 = cam.project(pt, &obs2_jac);
    std::cout << "project: " << obs2.transpose() << std::endl;
    std::cout << obs2_jac << std::endl;

    return EXIT_SUCCESS;
}
