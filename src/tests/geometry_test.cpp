#include <cstdlib>
#include <iostream>
#include <ceres_slam/geometry.h>

const double PI = 3.14159265;

// TODO: Rewrite this as a proper unit test

using SO3 = ceres_slam::SO3Group<double>;
using SE3 = ceres_slam::SE3Group<double>;

int main() {
    SO3 rot1;
    SO3::TangentVector phi(1.,0.5,PI/2);
    SO3 rot2 = SO3::exp(phi);
    SO3 rot3(rot2);
    SO3 rot4(rot2.matrix());
    std::cout << rot1 << std::endl << rot2 << std::endl
              << rot3 << std::endl << rot4 << std::endl;

    ceres_slam::Point3D<double> pt(0.,1.,0.);
    ceres_slam::Vector3D<double> vec(1.,0.,0.);
    ceres_slam::HomogeneousBase3D<double> h(0.,0.,1.,2.);

    std::cout << pt << std::endl << vec << std::endl << h << std::endl;

    std::cout << rot1 * pt << std::endl << rot2 * vec << std::endl
              << rot2 * rot2 << std::endl;

    std::cout << SO3::wedge(phi) << std::endl;
    std::cout << SO3::vee(SO3::wedge(phi)) << std::endl;
    std::cout << SO3::transformed_point_jacobian(rot2*pt) << std::endl;

    SE3 T1;
    SE3::TangentVector xi;
    xi << 0.1,0.2,0.3,1.,0.5,PI/2;
    SE3 T2(rot2, vec);
    SE3 T3(T2);
    SE3 T4(T2.matrix());
    std::cout << T1 << std::endl << T2 << std::endl
              << T3 << std::endl << T4 << std::endl;

    std::cout << T1 * pt << std::endl << T2 * vec << std::endl
              << T2 * T2 << std::endl;

    std::cout << SE3::wedge(xi) << std::endl;
    std::cout << SE3::vee(SE3::wedge(xi)) << std::endl;
    std::cout << SE3::transformed_point_jacobian(T2*pt) << std::endl;

    double xi2[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    SE3 T5 = SE3::exp(xi2);
    std::cout << T5 << std::endl;

    ceres_slam::Vector3D<double> vec2(1.,2.,3.);
    std::cout << vec2 << std::endl << vec2.norm() << std::endl;
    vec2.normalize();
    std::cout << vec2 << std::endl << vec2.norm() << std::endl;

    return EXIT_SUCCESS;
}
