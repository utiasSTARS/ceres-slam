#include <cstdlib>
#include <iostream>
#include <Eigen/Core>
#include <ceres_slam/geometry.h>

const double PI = 3.14159265;

// TODO: Rewrite this as a proper unit test

using Point = ceres_slam::Point3D<double>;
using Vector = ceres_slam::Vector3D<double>;
using SO3 = ceres_slam::SO3Group<double>;

int main() {
    ///////////////////////////////////////////////////////////////////////////
    // Point and vector tests
    ///////////////////////////////////////////////////////////////////////////
    Point p1;
    p1 << 1., 2., 3.;
    std::cout << "p1: " << p1 << std::endl;

    double p2_data[3] = {4., 5., 6.};
    Eigen::Map<Point> p2(&p2_data[0]);
    std::cout << "p2: " << p2 << std::endl;
    p2 += p1;
    std::cout << "p2: " << p2 << std::endl;
    std::cout << "p2_data: " << p2_data[0] << ", "
                             << p2_data[1] << ", "
                             << p2_data[2] << std::endl;

    Point p3 = p2;
    std::cout << "p3: " << p3 << std::endl;

    Vector v1;
    v1 << 1., 2., 3.;
    std::cout << "v1: " << v1 << std::endl;

    double v2_data[3] = {4., 5., 6.};
    Eigen::Map<Vector> v2(&v2_data[0]);
    std::cout << "v2: " << v2 << std::endl;
    v2 += v1;
    std::cout << "v2: " << v2 << std::endl;
    std::cout << "v2_data: " << v2_data[0] << ", "
                             << v2_data[1] << ", "
                             << v2_data[2] << std::endl;

    Vector v3 = v2;
    std::cout << "v3: " << v3 << std::endl;

    Point p4 = p2 + v3;
    std::cout << "p4: " << p4 << std::endl;

    Eigen::Map<const Point> p5(p4.data());
    std::cout << "p5: " << p5 << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    // SO(3) tests
    ///////////////////////////////////////////////////////////////////////////
    std::cout << std::endl;

    SO3 C1;
    std::cout << "C1: " << C1 << std::endl;

    SO3::TransformationMatrix C2_matrix;
    C2_matrix << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    SO3 C2(C2_matrix);
    std::cout << "C2: " << C2 << std::endl;

    SO3 C3(C2);
    std::cout << "C3: " << C3 << std::endl;
    std::cout << "C3.str(): " << C3.str() << std::endl;
    std::cout << "C3.inverse(): " << C3.inverse() << std::endl;

    return EXIT_SUCCESS;
}
