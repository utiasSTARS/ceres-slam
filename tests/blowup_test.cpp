#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include <ceres_slam/dataset_problem_sun.hpp>
#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/perturbations.hpp>
#include <ceres_slam/pose_error.hpp>
#include <ceres_slam/relative_pose_error.hpp>

#include <Eigen/Eigenvalues>

typedef ceres_slam::SE3Group<double> SE3;

int main() {
    uint num_poses = 10;

    std::vector<SE3> T_k_0;
    std::vector<SE3::AdjointMatrix> covars;

    T_k_0.resize(num_poses);
    covars.resize(num_poses);

    T_k_0[0] = SE3::Identity();
    covars[0] = 1e-6 * SE3::AdjointMatrix::Identity();

    SE3 meas = SE3::Identity();
    meas.translation()(0) = 0.1;
    SE3::AdjointMatrix meas_covar = 1e-2 * SE3::AdjointMatrix::Identity();
    Eigen::SelfAdjointEigenSolver<SE3::AdjointMatrix> es_meas(meas_covar);
    SE3::AdjointMatrix meas_stiffness = es_meas.operatorInverseSqrt();

    for (uint k1 = 0; k1 < num_poses - 1; ++k1) {
        ///////////////////////////////////////////////////////////////////////
        // Initial guess for k2
        uint k2 = k1 + 1;
        T_k_0[k2] = T_k_0[k1];
        std::cout << "[" << k1 << ", " << k2 << "]\n";

        ///////////////////////////////////////////////////////////////////////
        // Set up the problem
        ceres::Problem problem;

        Eigen::SelfAdjointEigenSolver<SE3::AdjointMatrix> es_prior(covars[k1]);
        SE3::AdjointMatrix prior_stiffness = es_prior.operatorInverseSqrt();

        // Add relative pose measurement from k1 to k2
        ceres::CostFunction *meas_cost =
            ceres_slam::RelativePoseErrorAutomatic::Create(meas,
                                                           meas_stiffness);
        problem.AddResidualBlock(meas_cost, NULL, T_k_0[k1].data(),
                                 T_k_0[k2].data());

        // Add prior on first pose
        // if (k1 == 0) {
        //     problem.SetParameterBlockConstant(T_k_0[k1].data());
        // } else {
        ceres::CostFunction *prior_cost =
            ceres_slam::PoseErrorAutomatic::Create(T_k_0[k1], prior_stiffness);
        problem.AddResidualBlock(prior_cost, NULL, T_k_0[k1].data());
        // }

        // Set local parameterizations
        ceres::LocalParameterization *se3_perturbation =
            ceres_slam::SE3Perturbation::Create();
        problem.SetParameterization(T_k_0[k1].data(), se3_perturbation);
        problem.SetParameterization(T_k_0[k2].data(), se3_perturbation);

        ///////////////////////////////////////////////////////////////////////
        // Solve
        ceres::Solver::Options solver_options;
        solver_options.minimizer_progress_to_stdout = false;
        solver_options.num_threads = 1;
        solver_options.num_linear_solver_threads = 1;
        solver_options.max_num_iterations = 1000;
        solver_options.use_nonmonotonic_steps = true;
        solver_options.trust_region_strategy_type = ceres::DOGLEG;
        solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
        // solver_options.check_gradients = true;

        ceres::Solver::Summary summary;

        Solve(solver_options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;

        ///////////////////////////////////////////////////////////////////////
        // Estimate covariance
        ceres::Covariance::Options covariance_options;
        covariance_options.num_threads = solver_options.num_threads;
        // covariance_options.algorithm_type = ceres::DENSE_SVD;
        covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;
        // covariance_options.null_space_rank = -1;

        ceres::Covariance covariance(covariance_options);

        std::vector<std::pair<const double *, const double *>> covar_blocks;
        covar_blocks.push_back(
            std::make_pair(T_k_0[k2].data(), T_k_0[k2].data()));

        if (!covariance.Compute(covar_blocks, &problem)) {
            std::cout << "WARNING: Covariance computation failed! "
                      << "Using previous state covariance." << std::endl;
            covars[k2] = covars[k1];
        } else {
            covariance.GetCovarianceBlockInTangentSpace(
                T_k_0[k2].data(), T_k_0[k2].data(), covars[k2].data());
        }

        std::cout << "\nCovariance for k=" << k2 << "\n" << covars[k2] << "\n";
    }

    std::cout << "Done.\n";

    return 0;
}
