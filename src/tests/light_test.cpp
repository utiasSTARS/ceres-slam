#include <cstdlib>
#include <iostream>
#include <memory>
#include <ceres_slam/geometry.h>
#include <ceres_slam/lighting.h>
#include <ceres_slam/intensity_error.h>

// TODO: Rewrite this as a proper unit test

using Point = ceres_slam::Point3D<double>;
using Vector = ceres_slam::Vector3D<double>;
using SE3 = ceres_slam::SE3Group<double>;
using Vertex = ceres_slam::Vertex3D<double>;
using Light = ceres_slam::PointLight<double>;
using Material = ceres_slam::Material<double>;

int main() {
    SE3::TransformationMatrix T_1_w_matrix;
    T_1_w_matrix <<
    0.99979182, -0.02040391,  0.        , -0.9793879 ,
    -0.00927144, -0.45430067, -0.89080017,  0.46357211,  0.01817581,
    0.89061472, -0.45439527,  1.29193662,
    0.        ,  0.        ,  0.        , 1.;
    SE3 T_1_w(T_1_w_matrix);
    Vector camera_position = Vector::Zero();

    Material::Ptr m = std::make_shared<Material>();
    m->phong_params() = Material::PhongParams(0.1, 0.3, 10.);
    double t = 0.6;

    Point v28_p;
    v28_p << 0.823015  ,  0.60803428,  0. ;
    // v28_p = T_1_w * v28_p;
    Vector v28_n;
    v28_n << 0.,0.,1.;
    // v28_n = T_1_w * v28_n;
    Vertex v28(v28_p, v28_n, m, t);
    Light::Colour v28_obs = 0.377606521;

    Vertex v245;
    v245.position() << 0.08868649,  1.,  0.7597348;
    // v245.position() = T_1_w * v245.position();
    v245.normal() << 0.,  -1.,  0.;
    // v245.normal() = T_1_w * v245.normal();
    v245.material() = m;
    v245.texture() = t;
    Light::Colour v245_obs = 0.777672166;

    Light l;
    l.position() << -2, -2, 2;
    // l.position() = T_1_w * l.position();
    Light::ColourJacobian l_jacobian;

    std::cout << "*m: " << *m << std::endl;
    std::cout << "v28: " << v28 << std::endl;
    // std::cout << "v28.str(): " << v28.str() << std::endl << std::endl;

    std::cout << "v245: " << v245 << std::endl;
    // std::cout << "v245.str(): " << v245.str() << std::endl << std::endl;

    std::cout << "camera_position: " << camera_position << std::endl << std::endl;
    std::cout << "l: " << l << std::endl;
    // std::cout << "l.str(): " << l.str() << std::endl << std::endl;

    std::cout << "l.shade(v28): " << l.shade(v28, camera_position, &l_jacobian)
              << std::endl;
    std::cout << "l.shade(v245): " << l.shade(v245, camera_position) << std::endl;

    // // Comment out Create function in intensity_error.h to use this code
    // std::cout << std::endl << "v28_err" << std::endl;
    //
    // double v28_err;
    // ceres_slam::IntensityErrorAutomatic v28_cost(v28_obs, 1.);
    // v28_cost(T_1_w.data(), v28.position().data(), v28.normal().data(),
    //          v28.material()->phong_params().data(), v28.texture_ptr(),
    //          l.position().data(), &v28_err);
    //
    // std::cout << std::endl << "v245_err" << std::endl;
    //
    // double v245_err;
    // ceres_slam::IntensityErrorAutomatic v245_cost(v245_obs, 1.);
    // v245_cost(T_1_w.data(), v245.position().data(), v245.normal().data(),
    //          v245.material()->phong_params().data(), v245.texture_ptr(),
    //          l.position().data(), &v245_err);

    return EXIT_SUCCESS;
}
