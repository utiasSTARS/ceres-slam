#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres/ceres.h>

#include <ceres_slam/dataset_problem.h>
#include <ceres_slam/stereo_reprojection_error.h>

#include <Eigen/Eigenvalues>

using SE3 = ceres_slam::DatasetProblem::SE3;
using Point = ceres_slam::DatasetProblem::Point;
using Camera = ceres_slam::DatasetProblem::Camera;

int main(int argc, char** argv) {
    if (argc < 2) {
      std::cerr << "usage: dataset_ba <input_file>"
                << std::endl;
      return EXIT_FAILURE;
    }

    // Read dataset from file
    std::string filename(argv[1]);
    ceres_slam::DatasetProblem dataset;
    if( !dataset.read_csv(filename) ) {
        return EXIT_FAILURE;
    }

    // Compute initial guess
    std::cerr << "Computing VO initial guess" << std::endl;
    dataset.compute_initial_guess();

    // Build the problem
    std::cerr << "Building problem" << std::endl;
    ceres::Problem problem;

    dataset.obs_var << 1, 1, 1; // u,v,d variance
    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Camera::ObservationCovariance>
        es(dataset.obs_var.asDiagonal());
    Camera::ObservationCovariance obs_stiffness = es.operatorInverseSqrt();

    for(unsigned int k = 0; k < dataset.num_states; ++k) {
        for(unsigned int i : dataset.obs_indices_at_state(k)) {
            // Map point ID for this observation
            unsigned int j = dataset.point_ids[i];
            // Only optimize map points that have been initialized
            if(dataset.initialized_point[j]) {
                // Cost function for this observation
                ceres::CostFunction* stereo_cost =
                    ceres_slam::StereoReprojectionErrorAnalytic::Create(
                        dataset.camera, dataset.obs_list[i], obs_stiffness);
                // ceres::CostFunction* stereo_cost =
                //     ceres_slam::StereoReprojectionErrorAutomatic::Create(
                //         dataset.camera, dataset.obs_list[i], obs_stiffness);
                // Add the cost function for this observation to the problem
                problem.AddResidualBlock(stereo_cost, NULL,
                                         dataset.pose_vectors[k].data(),
                                         dataset.map_points[j].data());
            }
        }
    }

    // Solve the problem
    std::cerr << "Solving" << std::endl;
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    // solver_options.linear_solver_type = ceres::DENSE_SCHUR;
    // solver_options.check_gradients = true;

    ceres::Solver::Summary summary;
    Solve(solver_options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    // Estimate covariance?

    // Output optimized state to file
    std::cerr << "Outputting to file " << std::endl;
    dataset.write_csv(filename);

    return EXIT_SUCCESS;
}
