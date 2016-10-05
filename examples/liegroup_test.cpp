#include <Eigen/Core>
#include <ceres_slam/geometry/geometry.hpp>
#include <cstdlib>
#include <iostream>

// TODO: Rewrite this as a proper unit test

using SO3 = ceres_slam::SO3Group<double>;
using SE3 = ceres_slam::SE3Group<double>;
using Vector = SO3::Vector;

void print_array(double* array, int n) {
    for (int i = 0; i < n; ++i) std::cout << *(array + i) << ", ";
    std::cout << std::endl;
}

int main() {
    ///////////////////////////////////////////////////////////////////////////
    // Vector tests
    ///////////////////////////////////////////////////////////////////////////
    Vector v1;
    v1 << 1., 2., 3.;
    std::cout << "v1: " << v1 << std::endl;

    double v2_data[3] = {4., 5., 6.};
    Eigen::Map<Vector> v2(&v2_data[0]);
    std::cout << "v2: " << v2 << std::endl;
    v2 += v1;
    std::cout << "v2: " << v2 << std::endl;
    std::cout << "v2_data: ";
    print_array(v2_data, 3);

    Vector v3 = v2;
    std::cout << "v3: " << v3 << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    // SO(3) tests
    ///////////////////////////////////////////////////////////////////////////
    std::cout << std::endl;

    SO3 C1;
    std::cout << "C1: " << C1 << std::endl;
    double* C1_data_ptr = C1.data();
    for (int i = 0; i < 9; ++i) {
        std::cout << C1_data_ptr[i] << " ";
    }
    std::cout << "\n\n";

    SO3::TransformationMatrix C2_matrix;
    C2_matrix << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    SO3 C2(C2_matrix);
    std::cout << "C2: " << C2 << std::endl;

    SO3 C3(C2);
    std::cout << "C3: " << C3 << std::endl;
    std::cout << "C3.str(): " << C3.str() << std::endl;
    std::cout << "C3.inverse(): " << C3.inverse() << std::endl;

    std::cout << "C2 * C3: " << C2 * C3 << std::endl;
    std::cout << "C2 * v1: " << C2 * v1 << std::endl;

    std::cout << "SO3::Identity(): " << SO3::Identity() << std::endl;
    std::cout << "SO3::log(SO3::Identity()): " << SO3::log(SO3::Identity())
              << std::endl;
    std::cout << "SO3::exp(SO3::log(SO3::Identity())): "
              << SO3::exp(SO3::log(SO3::Identity())) << std::endl;

    SO3::TangentVector phi1;
    phi1 << 1., 2., 3.;
    std::cout << "SO3::wedge(phi1): " << std::endl
              << SO3::wedge(phi1) << std::endl;
    std::cout << "SO3::vee(SO3::wedge(phi1)): " << std::endl
              << SO3::vee(SO3::wedge(phi1)) << std::endl;

    std::cout << "SO3::exp(phi1): " << SO3::exp(phi1) << std::endl;
    std::cout << "SO3::log(SO3::exp(phi1)): " << std::endl
              << SO3::log(SO3::exp(phi1)) << std::endl;
    std::cout << "SO3::exp(SO3::log(SO3::exp(phi1))): "
              << SO3::exp(SO3::log(SO3::exp(phi1))) << std::endl;

    double C4_data[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    Eigen::Map<SO3> C4(C4_data);
    std::cout << "C4: " << C4 << std::endl;
    C4 = C4 * C2;
    std::cout << "C4 = C4 * C2: " << C4 << std::endl;
    std::cout << "C4_data: ";
    print_array(C4_data, 9);
    std::cout << "C4 * v2: " << C4 * v2 << std::endl;

    SO3 C5;
    C5 = C4;
    C4 = C2;
    std::cout << "C5 = C4: " << C5 << std::endl;
    std::cout << "C4 = C2: " << C4 << std::endl;
    std::cout << "C4.adjoint(): " << std::endl << C4.adjoint() << std::endl;

    C4.normalize();
    std::cout << "C4.normalize(): " << C4 << std::endl;

    const double C6_data[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    Eigen::Map<const SO3> C6(C6_data);
    SO3::TransformJacobian C6p6_jac;
    std::cout << "C6: " << C6 << std::endl;
    std::cout << "C6.transform(p6): " << C6.transform(p6, &C6p6_jac)
              << std::endl;
    std::cout << C6p6_jac << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    // SE(3) tests
    ///////////////////////////////////////////////////////////////////////////
    std::cout << std::endl;

    SE3 T1;
    std::cout << "T1: " << T1 << std::endl;
    double* T1_data_ptr = T1.data();
    for (int i = 0; i < 12; ++i) {
        std::cout << T1_data_ptr[i] << " ";
    }
    std::cout << "\n\n";

    SE3::TransformationMatrix T2_matrix;
    T2_matrix << 0, -1, 0, 1, 1, 0, 0, -1, 0, 0, 1, 1, 0, 0, 0, 1;
    SE3 T2(T2_matrix);
    std::cout << "T2: " << T2 << std::endl;

    SE3 T3(T2);
    std::cout << "T3: " << T3 << std::endl;
    std::cout << "T3.str(): " << T3.str() << std::endl;
    std::cout << "T3.inverse(): " << T3.inverse() << std::endl;
    std::cout << "T3.data(): ";
    print_array(T3.data(), 12);

    std::cout << "T2 * T3: " << T2 * T3 << std::endl;
    std::cout << "T2 * v1: " << T2 * v1 << std::endl;

    std::cout << "SE3::Identity(): " << SE3::Identity() << std::endl;
    std::cout << "SE3::log(SE3::Identity()): " << SE3::log(SE3::Identity())
              << std::endl;
    std::cout << "SE3::exp(SE3::log(SE3::Identity())): "
              << SE3::exp(SE3::log(SE3::Identity())) << std::endl;

    SE3::TangentVector xi1;
    xi1 << 1., 2., 3., 4., 5., 6.;
    std::cout << "SE3::wedge(xi1): " << std::endl
              << SE3::wedge(xi1) << std::endl;
    std::cout << "SE3::vee(SE3::wedge(xi1)): " << std::endl
              << SE3::vee(SE3::wedge(xi1)) << std::endl;

    std::cout << "SE3::exp(xi1): " << SE3::exp(xi1) << std::endl;
    std::cout << "SE3::log(SE3::exp(xi1)): " << std::endl
              << SE3::log(SE3::exp(xi1)) << std::endl;
    std::cout << "SE3::exp(SE3::log(SE3::exp(xi1))): "
              << SE3::exp(SE3::log(SE3::exp(xi1))) << std::endl;

    double T4_data[12] = {1, -1, 1, 0, -1, 0, 1, 0, 0, 0, 0, 1};
    Eigen::Map<SE3> T4(T4_data);
    std::cout << "T4: " << T4 << std::endl;
    T4 = T4 * T2;
    std::cout << "T4 = T4 * T2: " << T4 << std::endl;
    std::cout << "T4_data: ";
    print_array(T4_data, 12);
    std::cout << "T4 * v2: " << T4 * v2 << std::endl;
    std::cout << "T4.inverse(): " << T4.inverse() << std::endl;
    std::cout << "T4.adjoint(): " << std::endl << T4.adjoint() << std::endl;

    T4.normalize();
    std::cout << "T4.normalize(): " << T4 << std::endl;

    const double T6_data[12] = {1, -1, 1, 0, -1, 0, 1, 0, 0, 0, 0, 1};
    Eigen::Map<const SE3> T6(T6_data);
    SE3::TransformJacobian T6v1_jac;
    std::cout << "T6: " << T6 << std::endl;
    std::cout << "T6.transform(v1): " << T6.transform(v1, &T6v1_jac)
              << std::endl;
    std::cout << T6v1_jac << std::endl;

    SE3::TransformationMatrix T_0_w_matrix, T_1_0_ceres_matrix,
        T_1_w_ceres_matrix;
    T_0_w_matrix << 1, -0, 0, -1, 0, -0.4472, -0.8944, 0.4472, 0, 0.8944,
        -0.4472, 1.342, 0, 0, 0, 1;
    T_1_0_ceres_matrix << 0.9998, 0.009125, -0.01825, 0.04081, -0.009271,
        0.9999, -0.007961, 0.0178, 0.01818, 0.008128, 0.9998, -0.0349, 0, 0, 0,
        1;
    T_1_w_ceres_matrix << 0.9995, -0.02937, 0.009072, -0.9472, -0.005199,
        -0.4525, -0.8918, 0.4422, 0.03029, 0.8913, -0.4524, 1.35, 0, 0, 0, 1;

    SO3 C_0_w(T_0_w_matrix.block<3, 3>(0, 0));
    SO3 C_1_0(T_1_0_ceres_matrix.block<3, 3>(0, 0));
    SO3 C_1_w(T_1_w_ceres_matrix.block<3, 3>(0, 0));

    std::cout << "C_0_w: " << C_0_w << std::endl;
    std::cout << "C_1_0: " << C_1_0 << std::endl;
    std::cout << "C_1_w: " << C_1_w << std::endl;

    std::cout << "C_1_0.matrix() * C_0_w.matrix(): " << std::endl
              << C_1_0.matrix() * C_0_w.matrix() << std::endl;
    std::cout << "C_1_0 * C_0_w: " << std::endl << C_1_0 * C_0_w << std::endl;
    std::cout
        << "T_1_0_ceres_matrix.block<3,3>(0,0) * T_0_w_matrix.block<3,3>(0,0): "
        << std::endl
        << T_1_0_ceres_matrix.block<3, 3>(0, 0) * T_0_w_matrix.block<3, 3>(0, 0)
        << std::endl;

    SE3 T_0_w(T_0_w_matrix);
    SE3 T_1_0(T_1_0_ceres_matrix);
    SE3 T_1_w(T_1_w_ceres_matrix);

    std::cout << "T_0_w: " << T_0_w << std::endl;
    std::cout << "T_1_0: " << T_1_0 << std::endl;
    std::cout << "T_1_w: " << T_1_w << std::endl;
    std::cout << "T_1_0_ceres_matrix * T_0_w_matrix: " << std::endl
              << T_1_0_ceres_matrix * T_0_w_matrix << std::endl;
    std::cout << "T_1_0 * T_0_w: " << std::endl << T_1_0 * T_0_w << std::endl;

    std::cout << "T_1_w * T_1_w.inverse(): " << T_1_w * T_1_w.inverse()
              << std::endl;
    std::cout << "T_1_w.inverse() * T_1_w: " << T_1_w.inverse() * T_1_w
              << std::endl;

    return EXIT_SUCCESS;
}
