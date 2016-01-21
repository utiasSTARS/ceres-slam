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
    m->phong_params() = Material<double>::PhongParams(0.3, 0.7);

    Vertex3D<double> v;
    v.position() << 1.,1.,1.;
    v.normal() << 0.,0.,1.;
    v.material() = m;

    PointLight<double> l;
    l.position() << 5., 5., 5.;
    l.phong_params() = PointLight<double>::PhongParams(0.9, 0.9);
    PointLight<double>::ColourJacobian l_jacobian;

    std::cout << *m << std::endl;
    std::cout << v << std::endl;
    std::cout << v.str() << std::endl << std::endl;
    std::cout << l << std::endl;
    std::cout << l.str() << std::endl << std::endl;
    std::cout << l.shade(v, &l_jacobian) << std::endl;
    std::cout << l_jacobian << std::endl;

    return EXIT_SUCCESS;
}
