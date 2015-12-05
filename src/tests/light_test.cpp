#include <cstdlib>
#include <iostream>
#include <ceres_slam/geometry.h>
#include <ceres_slam/point_light.h>

// TODO: Rewrite this as a proper unit test

using namespace ceres_slam;

int main() {
    Vertex3D<double> v( Point3D<double>(1.,1.,1.),
                        Vector3D<double>(0.,0.,1.),
                        0.3, 0.7 );

    PointLight<double> l( Point3D<double>(5., 5., 5.) );

    std::cout << v << std::endl;
    std::cout << l << std::endl;
    std::cout << l.shade(v) << std::endl;

    return EXIT_SUCCESS;
}
