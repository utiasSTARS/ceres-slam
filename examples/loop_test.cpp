#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include <Eigen/Eigenvalues>

#include "../src/cost_functions/pose_cost.hpp"
#include "../src/cost_functions/relative_pose_cost.hpp"
#include "../src/liegroups/se3group.hpp"
#include "../src/local_parameterizations/se3_local_param.hpp"
#include "../src/utils/utils.hpp"

typedef ceres_slam::SO3Group<double> SO3;
typedef ceres_slam::SE3Group<double> SE3;

int main() {
    // True
    SE3 T_1_0_true, T_2_0_true, T_3_0_true, T_4_0_true, T_5_0_true, T_6_0_true;

    T_1_0_true = SE3::Identity();
    T_2_0_true = SE3(-SE3::Vector(0.5, 0, 0), SO3::Identity());
    T_3_0_true = SE3(-SE3::Vector(1, 0, 0), SO3::Identity());
    T_4_0_true = SE3(-(SO3::exp(SO3::TangentVector(0, 0, ceres_slam::pi / 2)) *
                       SE3::Vector(1, 0.5, 0)),
                     SO3::exp(SO3::TangentVector(0, 0, ceres_slam::pi / 2)));
    T_5_0_true = SE3(-(SO3::exp(SO3::TangentVector(0, 0, ceres_slam::pi)) *
                       SE3::Vector(0.5, 0.5, 0)),
                     SO3::exp(SO3::TangentVector(0, 0, ceres_slam::pi)));
    T_6_0_true = SE3(-(SO3::exp(SO3::TangentVector(0, 0, -ceres_slam::pi / 2)) *
                       SE3::Vector(0.5, 0, 0)),
                     SO3::exp(SO3::TangentVector(0, 0, -ceres_slam::pi / 2)));

    // Odometry
    SE3 T_1_0_obs, T_2_1_obs, T_3_2_obs, T_4_3_obs, T_5_4_obs, T_6_5_obs;

    T_1_0_obs = SE3::Identity();
    T_2_1_obs = T_2_0_true * T_1_0_true.inverse();
    T_3_2_obs = T_3_0_true * T_2_0_true.inverse();
    T_4_3_obs = T_4_0_true * T_3_0_true.inverse();
    T_5_4_obs = T_5_0_true * T_4_0_true.inverse();
    T_6_5_obs = T_6_0_true * T_5_0_true.inverse();

    // Loop closure
    SE3 T_6_2_obs = T_6_0_true * T_2_0_true.inverse();

    // Constant offset start
    SE3 T_1_0_est, T_2_0_est, T_3_0_est, T_4_0_est, T_5_0_est, T_6_0_est;

    SE3::TangentVector offset1, offset2;
    offset1 << -0.1, 0.1, -0.1, 0.1, -0.1, 0.1;
    offset2 << 0.1, -0.1, 0.1, -0.1, 0.1, -0.1;

    T_1_0_est = T_1_0_true;
    T_2_0_est = SE3::exp(offset1) * T_2_0_true;
    T_3_0_est = SE3::exp(offset2) * T_3_0_true;
    T_4_0_est = SE3::exp(offset1) * T_4_0_true;
    T_5_0_est = SE3::exp(offset2) * T_5_0_true;
    T_6_0_est = SE3::exp(offset1) * T_6_0_true;

    SE3::AdjointMatrix obs_covar = SE3::AdjointMatrix::Identity();
    Eigen::SelfAdjointEigenSolver<SE3::AdjointMatrix> es_obs(obs_covar);
    SE3::AdjointMatrix obs_stiffness = es_obs.operatorInverseSqrt();

    // Problem setup
    ceres::Problem problem;

    ceres::CostFunction *cost1, *cost2, *cost3, *cost4, *cost5, *cost6;
    cost1 =
        ceres_slam::RelativePoseCostAutomatic::Create(T_2_1_obs, obs_stiffness);
    cost2 =
        ceres_slam::RelativePoseCostAutomatic::Create(T_3_2_obs, obs_stiffness);
    cost3 =
        ceres_slam::RelativePoseCostAutomatic::Create(T_4_3_obs, obs_stiffness);
    cost4 =
        ceres_slam::RelativePoseCostAutomatic::Create(T_5_4_obs, obs_stiffness);
    cost5 =
        ceres_slam::RelativePoseCostAutomatic::Create(T_6_5_obs, obs_stiffness);
    cost6 =
        ceres_slam::RelativePoseCostAutomatic::Create(T_6_2_obs, obs_stiffness);

    problem.AddResidualBlock(cost1, NULL, T_2_0_est.data(), T_1_0_est.data());
    problem.AddResidualBlock(cost2, NULL, T_3_0_est.data(), T_2_0_est.data());
    problem.AddResidualBlock(cost3, NULL, T_4_0_est.data(), T_3_0_est.data());
    problem.AddResidualBlock(cost4, NULL, T_5_0_est.data(), T_4_0_est.data());
    problem.AddResidualBlock(cost5, NULL, T_6_0_est.data(), T_5_0_est.data());
    problem.AddResidualBlock(cost6, NULL, T_6_0_est.data(), T_2_0_est.data());

    ceres::LocalParameterization *se3_local_param =
        ceres_slam::SE3LocalParameterization::Create();

    problem.SetParameterization(T_1_0_est.data(), se3_local_param);
    problem.SetParameterization(T_2_0_est.data(), se3_local_param);
    problem.SetParameterization(T_3_0_est.data(), se3_local_param);
    problem.SetParameterization(T_4_0_est.data(), se3_local_param);
    problem.SetParameterization(T_5_0_est.data(), se3_local_param);
    problem.SetParameterization(T_6_0_est.data(), se3_local_param);

    problem.SetParameterBlockConstant(T_1_0_est.data());

    // Solve
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.num_threads = 1;
    solver_options.num_linear_solver_threads = 1;
    solver_options.max_num_iterations = 1000;
    solver_options.use_nonmonotonic_steps = true;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // solver_options.check_gradients = true;

    ceres::Solver::Summary summary;

    Solve(solver_options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    return 0;
}
