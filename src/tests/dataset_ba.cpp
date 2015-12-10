#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <ceres_slam/dataset_problem.h>
#include <ceres_slam/point_cloud_aligner.h>

int main(int argc, char** argv) {
    if (argc < 2) {
      std::cerr << "usage: dataset_ba <input_file> <optional: output_file>"
                << std::endl;
      return EXIT_FAILURE;
    }

    // Read dataset from file
    ceres_slam::DatasetProblem dataset;
    dataset.read_csv(std::string(argv[1]));

    // Generate initial guess using scalar-weighted point cloud alignment

    // Build the problem
    ceres::Problem problem;

    // Solve the problem
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

    ceres::Solver::Summary summary;
    Solve(solver_options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    // Estimate covariance?

    // Output optimized state to file

    return EXIT_SUCCESS;
}
