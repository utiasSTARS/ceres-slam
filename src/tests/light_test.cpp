#include <cstdlib>
#include <iostream>
#include <memory>
#include <ceres_slam/geometry.h>
#include <ceres_slam/point_light.h>
#include <ceres_slam/material.h>

// TODO: Rewrite this as a proper unit test

using namespace ceres_slam;

int main() {
    Material<double>::Ptr m = std::make_shared<Material<double>>();
    m->phong_params() = Material<double>::PhongParams(0.1, 0.9, 0.5, 4.);

    Point3D<double> v6_p;
    v6_p << 0.39151551, -0.04951286,  0. ;
    Vector3D<double> v6_n;
    v6_n << 0.,0.,1.;
    Vertex3D<double> v6(v6_p, v6_n, m);

    Vertex3D<double> v118;
    v118.position() << 1., 0.88091717, 0.09214285;
    v118.normal() << -1.,  0.,  0.;
    v118.material() = m;

    PointLight<double> l;
    l.position() << 0.5, 0.5, 0.5;
    PointLight<double>::ColourJacobian l_jacobian;

    Vector3D<double> camera_position;
    camera_position <<  1., -1.,  1.;

    std::cout << "*m: " << *m << std::endl;
    std::cout << "v6: " << v6 << std::endl;
    std::cout << "v6.str(): " << v6.str() << std::endl << std::endl;
    std::cout << "v118: " << v118 << std::endl;
    std::cout << "v118.str(): " << v118.str() << std::endl << std::endl;
    std::cout << "l: " << l << std::endl;
    std::cout << "l.str(): " << l.str() << std::endl << std::endl;
    std::cout << "l.shade(v6): " << l.shade(v6, camera_position, &l_jacobian)
              << std::endl;
    std::cout << "l.shade(v118): " << l.shade(v118, camera_position, &l_jacobian)
              << std::endl;

    return EXIT_SUCCESS;
}
