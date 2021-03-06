#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <ceres/ceres.h>

#include <ceres_slam/dataset_problem.hpp>
#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/perturbations.hpp>
#include <ceres_slam/stereo_camera.hpp>
#include <ceres_slam/stereo_reprojection_error.hpp>
#include <ceres_slam/utils/utils.hpp>

#include <Eigen/Eigenvalues>

using SE3 = ceres_slam::DatasetProblem::SE3;
using Point = ceres_slam::DatasetProblem::Point;
using Vector = ceres_slam::DatasetProblem::Vector;
using Camera = ceres_slam::DatasetProblem::Camera;

void solveWindow(ceres_slam::DatasetProblem &dataset, uint k1, uint k2) {
    // Build the problem
    std::cerr << "Working on interval [" << k1 << "," << k2 << ")" << std::endl;

    ceres::Problem problem;

    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Camera::ObservationCovariance> es_stereo(
        dataset.stereo_obs_var.asDiagonal());
    Camera::ObservationCovariance stereo_obs_stiffness =
        es_stereo.operatorInverseSqrt();

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

        // Use local parameterization on SE(3)
        problem.SetParameterization(dataset.poses[k].data(), se3_perturbation);
    }

    // Hold the first pose constant
    problem.SetParameterBlockConstant(dataset.poses[k1].data());

    // Set solver options
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = false;
    solver_options.num_threads = 8;
    solver_options.num_linear_solver_threads = 8;
    solver_options.max_num_iterations = 1000;
    solver_options.use_nonmonotonic_steps = true;
    // solver_options.trust_region_strategy_type = ceres::DOGLEG;
    // solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    // solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // solver_options.check_gradients = true;

    // Create sumary container
    ceres::Solver::Summary summary;

    ///////////////////////////////////////////////////////////////////////
    // Optimize!
    Solve(solver_options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl << std::endl;

    // Estimate covariance?
}

int main(int argc, char **argv) {
    std::string usage_string("usage: dataset_vo <input_file> [--window N=0]");

    if (argc < 2) {
        std::cerr << usage_string << std::endl;
        return EXIT_FAILURE;
    }

    // Defaults
    uint window_size = 0;

    // Parse command line arguments
    std::string filename(argv[1]);
    for (int a = 2; a < argc; ++a) {
        std::string flag(argv[a]);

        if (flag == "--window" && argc > a + 1) {
            window_size = std::atoi(argv[a + 1]);
            ++a;
        } else {
            std::cerr << usage_string << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Read dataset from file
    ceres_slam::DatasetProblem dataset;
    if (!dataset.read_csv(filename)) {
        return EXIT_FAILURE;
    }

    // Special case: window_size == 0 means full batch
    if (window_size == 0) {
        window_size = dataset.num_states;
    }

    // Compute initial guess
    std::cerr << "Computing VO" << std::endl;
    for (uint k1 = 0; k1 <= dataset.num_states - window_size; ++k1) {
        uint k2 = fmin(k1 + window_size, dataset.num_states);
        // std::cout << "k1 = " << k1 << ", k2 = " << k2 << std::endl;
        dataset.compute_initial_guess(k1, k2);
        solveWindow(dataset, k1, k2);
        dataset.reset_points();
    }

    // Output optimized state to file
    std::cerr << "Outputting to file " << std::endl;
    dataset.write_csv(filename);

    return EXIT_SUCCESS;
}
