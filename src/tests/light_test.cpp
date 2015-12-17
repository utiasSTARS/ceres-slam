#include <cstdlib>
#include <iostream>
#include <ceres_slam/geometry.h>
#include <ceres_slam/point_light.h>

// TODO: Rewrite this as a proper unit test

using namespace ceres_slam;

int main() {
    Vertex3D<double> v;
    v.position() = Point3D<double>(1.,1.,1.);
    v.normal() = Vector3D<double>(0.,0.,1.);
    v.phong_params() = Vertex3D<double>::PhongParams(0.3, 0.7);

    PointLight<double> l;
    l.position() = Point3D<double>(5., 5., 5.);
    l.phong_params() = PointLight<double>::PhongParams(0.9, 0.9);
    PointLight<double>::ColourJacobian l_jacobian;

    std::cout << v << std::endl;
    std::cout << l << std::endl;
    std::cout << l.shade(v, &l_jacobian) << std::endl;
    std::cout << l_jacobian << std::endl;

    return EXIT_SUCCESS;
}
