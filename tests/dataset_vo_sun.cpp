#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <ceres/ceres.h>

#include <ceres_slam/dataset_problem_sun.hpp>
#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/perturbations.hpp>
#include <ceres_slam/pose_error.hpp>
#include <ceres_slam/stereo_camera.hpp>
#include <ceres_slam/stereo_reprojection_error.hpp>
#include <ceres_slam/sun_sensor_error.hpp>
#include <ceres_slam/utils/utils.hpp>

#include <Eigen/Eigenvalues>

using SE3 = ceres_slam::DatasetProblemSun::SE3;
using Point = ceres_slam::DatasetProblemSun::Point;
using Vector = ceres_slam::DatasetProblemSun::Vector;
using Camera = ceres_slam::DatasetProblemSun::Camera;

void solveWindow(ceres_slam::DatasetProblemSun &dataset, uint k1, uint k2,
                 bool use_sun) {
    // Build the problem
    std::cerr << "Working on interval [" << k1 << "," << k2 << ")/"
              << dataset.num_states << ": ";

    std::cout << "\n\nCovariance for k=" << k1 << "\n"
              << dataset.pose_covars[k1] << std::endl;

    ceres::Problem problem;

    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Camera::ObservationCovariance> es_stereo(
        dataset.stereo_obs_var.asDiagonal());
    Camera::ObservationCovariance stereo_obs_stiffness =
        es_stereo.operatorInverseSqrt();

    Eigen::SelfAdjointEigenSolver<Vector::Covariance> es_sun(
        dataset.sun_obs_var.asDiagonal());
    Vector::Covariance sun_obs_stiffness = es_sun.operatorInverseSqrt();

    // Set up local parameterizations
    ceres::LocalParameterization *se3_perturbation =
        ceres_slam::SE3Perturbation::Create();

    // Add observations and cost functions
    for (uint k = k1; k < k2; ++k) {
        for (uint i : dataset.obs_indices_at_state(k)) {
            // Map point ID for this observation
            uint j = dataset.point_ids[i];
            // Only optimize map points that have been initialized
            if (dataset.initialized_point[j]) {
                // Cost function for the stereo observation
                ceres::CostFunction *stereo_cost =
                    ceres_slam::StereoReprojectionErrorAutomatic::Create(
                        dataset.camera, dataset.stereo_obs_list[i],
                        stereo_obs_stiffness);
                // Add the stereo cost function to the problem
                problem.AddResidualBlock(stereo_cost, NULL,
                                         dataset.poses[k].data(),
                                         dataset.map_points[j].data());
            }
        }

        // Add sun sensor measurements if available
        if (use_sun && dataset.state_has_sun_obs[k]) {
            // Cost function for the sun observation
            ceres::CostFunction *sun_cost =
                ceres_slam::SunSensorErrorAutomatic::Create(
                    dataset.sun_obs_list[k], dataset.sun_dir_g,
                    sun_obs_stiffness);
            // Add the sun sensor cost function to the problem
            problem.AddResidualBlock(sun_cost, NULL, dataset.poses[k].data());
        }
    }

    // Hold the initial pose in the sequence constant
    // problem.SetParameterBlockConstant(dataset.poses[k1].data());

    // Add prior on the first pose in the current window based on the
    // result from the previous window
    Eigen::SelfAdjointEigenSolver<SE3::AdjointMatrix> es_prior(
        dataset.pose_covars[k1]);
    // Eigen::SelfAdjointEigenSolver<SE3::AdjointMatrix> es_prior(
    //     1e-2 * SE3::AdjointMatrix::Identity());

    SE3::AdjointMatrix pose_prior_stiffness = es_prior.operatorInverseSqrt();

    // std::cout << "\nStiffness for k=" << k1 << "\n"
    //           << pose_prior_stiffness << std::endl;

    ceres::CostFunction *pose_prior_cost =
        ceres_slam::PoseErrorAutomatic::Create(dataset.poses[k1],
                                               pose_prior_stiffness);
    problem.AddResidualBlock(pose_prior_cost, NULL, dataset.poses[k1].data());

    // Use local parameterization on SE(3) for all poses
    for (uint k = k1; k < k2; ++k) {
        problem.SetParameterization(dataset.poses[k].data(), se3_perturbation);
    }

    ///////////////////////////////////////////////////////////////////////
    // Set up the solver and solve!

    // Set solver options
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = false;
    solver_options.num_threads = 8;
    solver_options.num_linear_solver_threads = 8;
    solver_options.max_num_iterations = 1000;
    solver_options.use_nonmonotonic_steps = true;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    // solver_options.check_gradients = true;

    // Create sumary container
    ceres::Solver::Summary summary;

    // Optimize!
    Solve(solver_options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    ///////////////////////////////////////////////////////////////////////
    // Estimate covariance for the second pose in the window so we can use
    // it as a prior on the first pose in the next window
    // std::cout << "Estimating covariance" << std::endl;

    ceres::Covariance::Options covariance_options;
    covariance_options.num_threads = solver_options.num_threads;
    // covariance_options.algorithm_type = ceres::DENSE_SVD;
    covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;
    // covariance_options.null_space_rank = -1;

    ceres::Covariance covariance(covariance_options);

    std::vector<std::pair<const double *, const double *>> covar_blocks;
    covar_blocks.push_back(std::make_pair(dataset.poses[k1 + 1].data(),
                                          dataset.poses[k1 + 1].data()));

    if (!covariance.Compute(covar_blocks, &problem)) {
        std::cout << "WARNING: Covariance computation failed! "
                  << "Using previous state covariance." << std::endl;
        dataset.pose_covars[k1 + 1] = dataset.pose_covars[k1];
    } else {
        covariance.GetCovarianceBlockInTangentSpace(
            dataset.poses[k1 + 1].data(), dataset.poses[k1 + 1].data(),
            dataset.pose_covars[k1 + 1].data());
    }

    Eigen::LLT<SE3::AdjointMatrix> llt_of_covar(dataset.pose_covars[k1 + 1]);
    if (llt_of_covar.info() == Eigen::NumericalIssue) {
        std::cout
            << "WARNING: Covariance matrix is not positive semi-definite! "
            << "Using previous state covariance." << std::endl;
        dataset.pose_covars[k1 + 1] = dataset.pose_covars[k1];
    }
}

int main(int argc, char **argv) {
    std::string usage_string(
        "usage: dataset_vo_sun <input_file> [--window N=0]");

    if (argc < 2) {
        std::cerr << usage_string << std::endl;
        return EXIT_FAILURE;
    }

    // Defaults
    uint window_size = 0;
    bool sun_only = false;

    // Parse command line arguments
    std::string filename(argv[1]);
    for (int a = 2; a < argc; ++a) {
        std::string flag(argv[a]);

        if (flag == "--window" && argc > a + 1) {
            window_size = std::atoi(argv[a + 1]);
            ++a;
        } else if (flag == "--sun-only") {
            sun_only = true;
        } else {
            std::cerr << usage_string << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Read dataset from file
    ceres_slam::DatasetProblemSun dataset;
    if (!dataset.read_csv(filename)) {
        return EXIT_FAILURE;
    }

    // Special case: window_size == 0 means full batch
    if (window_size == 0) {
        window_size = dataset.num_states;
    }

    // Compute initial guess
    if (!sun_only) {
        std::cerr << "Computing VO without sun measurements" << std::endl;
        for (uint k1 = 0; k1 <= dataset.num_states - window_size; ++k1) {
            uint k2 = fmin(k1 + window_size, dataset.num_states);
            // std::cout << "k1 = " << k1 << ", k2 = " << k2 << std::endl;
            dataset.compute_initial_guess(k1, k2);
            solveWindow(dataset, k1, k2, false);
            dataset.reset_points();
        }

        // Output the initial guess to a CSV file for comparison
        std::vector<std::string> tokens;
        tokens = ceres_slam::split(filename, '.');
        dataset.write_csv(tokens.at(0) + "_initial.csv");
    }

    std::cerr << "Computing VO with sun measurements" << std::endl;
    for (uint k1 = 0; k1 <= dataset.num_states - window_size; ++k1) {
        uint k2 = fmin(k1 + window_size, dataset.num_states);
        // std::cout << "k1 = " << k1 << ", k2 = " << k2 << std::endl;
        dataset.compute_initial_guess(k1, k2);
        solveWindow(dataset, k1, k2, true);
        dataset.reset_points();
    }

    // Output optimized state to file
    std::cerr << "Outputting to file: " << filename << std::endl;
    dataset.write_csv(filename);

    return EXIT_SUCCESS;
}
