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
    std::cout << T2*pt << std::endl;
    std::cout << SE3::odot(T2*pt) << std::endl;


    ceres_slam::Vector3D<double> vec2(1.,2.,3.);
    std::cout << vec2 << std::endl << vec2.norm() << std::endl;
    vec2.normalize();
    std::cout << vec2 << std::endl << vec2.norm() << std::endl;

    std::cout << rot1.str() << std::endl;
    std::cout << pt.str() << std::endl;
    std::cout << vec2.str() << std::endl;
    std::cout << T2.str() << std::endl;

    std::cout << rot2 * rot2.inverse() << std::endl;
    std::cout << T2 * T2.inverse() << std::endl;

    SO3::TangentVector phi1, phi2;
    phi1 << 1.,2.,3.;
    SO3 C1 = SO3::exp(phi1);
    phi2 = SO3::log(C1);
    SO3 C2 = SO3::exp(phi2);
    std::cout << "phi1 = " << phi1.transpose() << std::endl;
    std::cout << "C1 = " << std::endl<< C1 << std::endl;
    std::cout << "phi2 = " << phi2.transpose() << std::endl;
    std::cout << "C2 = " << std::endl<< C2 << std::endl;
    std::cout << "C1.inverse() * C2 = " << std::endl<< C1.inverse() * C2 << std::endl;

    SE3::TangentVector xia, xib;
    xia << 1.,2.,3.,4.,5.,6.;
    SE3 Ta = SE3::exp(xia);
    xib = SE3::log(Ta);
    SE3 Tb = SE3::exp(xib);
    std::cout << "xia = " << xia.transpose() << std::endl;
    std::cout << "Ta = " << std::endl<< Ta << std::endl;
    std::cout << "xib = " << xib.transpose() << std::endl;
    std::cout << "Tb = " << std::endl<< Tb << std::endl;
    std::cout << "Ta.inverse() * Tb = " << std::endl<< Ta.inverse() * Tb << std::endl;


    return EXIT_SUCCESS;
}
