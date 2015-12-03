#include <cstdlib>
#include <iostream>
#include <ceres_slam/geometry.h>

const double PI = 3.14159265;

int main() {
    ceres_slam::SO3<double> rot1;
    ceres_slam::SO3<double>::TangentVector phi(1.,0.5,PI/2);
    ceres_slam::SO3<double> rot2 = ceres_slam::SO3<double>::exp(phi);
    ceres_slam::SO3<double> rot3(rot2);
    ceres_slam::SO3<double> rot4(rot2.matrix());
    std::cout << rot1 << std::endl << rot2 << std::endl
              << rot3 << std::endl << rot4 << std::endl;

    ceres_slam::Point3D<double> pt(0.,1.,0.);
    ceres_slam::Vector3D<double> vec(1.,0.,0.);
    ceres_slam::Homogeneous3D<double> h(0.,0.,1.,2.);

    std::cout << pt << std::endl << vec << std::endl << h << std::endl;

    std::cout << rot1 * pt << std::endl << rot2 * vec << std::endl
              << rot2 * rot2 << std::endl;

    std::cout << ceres_slam::SO3<double>::wedge(phi) << std::endl;
    std::cout << ceres_slam::SO3<double>::vee(ceres_slam::SO3<double>::wedge(phi)) << std::endl;
    std::cout << ceres_slam::SO3<double>::transformed_point_jacobian(rot2*pt) << std::endl;

    ceres_slam::SE3<double> T1;
    ceres_slam::SE3<double>::TangentVector xi;
    xi << 0.1,0.2,0.3,1.,0.5,PI/2;
    ceres_slam::SE3<double> T2(rot2, vec);
    ceres_slam::SE3<double> T3(T2);
    ceres_slam::SE3<double> T4(T2.matrix());
    std::cout << T1 << std::endl << T2 << std::endl
              << T3 << std::endl << T4 << std::endl;

    std::cout << T1 * pt << std::endl << T2 * vec << std::endl
              << T2 * T2 << std::endl;

    std::cout << ceres_slam::SE3<double>::wedge(xi) << std::endl;
    std::cout << ceres_slam::SE3<double>::vee(ceres_slam::SE3<double>::wedge(xi)) << std::endl;
    std::cout << ceres_slam::SE3<double>::transformed_point_jacobian(T2*pt) << std::endl;

    return EXIT_SUCCESS;
}
