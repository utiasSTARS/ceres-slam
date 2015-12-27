#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres/ceres.h>

#include <ceres_slam/dataset_problem.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/stereo_reprojection_error.h>
#include <ceres_slam/point_light.h>
#include <ceres_slam/intensity_error.h>

#include <Eigen/Eigenvalues>

using SE3 = ceres_slam::DatasetProblem::SE3;
using Point = ceres_slam::DatasetProblem::Point;
using Camera = ceres_slam::DatasetProblem::Camera;
using Light = ceres_slam::DatasetProblem::Light;

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

    dataset.obs_var << 1., 1., 1.; // u,v,d variance
    dataset.int_var = 0.0001; // I variance

    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Camera::ObservationCovariance>
        es(dataset.obs_var.asDiagonal());
    Camera::ObservationCovariance obs_stiffness = es.operatorInverseSqrt();

    Light::ColourCovariance int_stiffness = 1. / sqrt(dataset.int_var);

    for(unsigned int k = 0; k < dataset.num_states; ++k) {
        for(unsigned int i : dataset.obs_indices_at_state(k)) {
            // Map point ID for this observation
            unsigned int j = dataset.vertex_ids[i];
            // Only optimize map points that have been initialized
            if(dataset.initialized_vertex[j]) {
                // Cost function for the stereo observation
                // ceres::CostFunction* stereo_cost =
                //     ceres_slam::StereoReprojectionErrorAnalytic::Create(
                //         dataset.camera, dataset.obs_list[i], obs_stiffness);
                ceres::CostFunction* stereo_cost =
                    ceres_slam::StereoReprojectionErrorAutomatic::Create(
                        dataset.camera, dataset.obs_list[i], obs_stiffness);
                // Add the stereo cost function to the problem
                problem.AddResidualBlock(
                    stereo_cost, NULL,
                    dataset.pose_vectors[k].data(),
                    dataset.map_vertices[j].position().data() );

                // Cost function for the intensity observation
                ceres::CostFunction* intensity_cost =
                    ceres_slam::IntensityErrorAutomatic::Create(
                        dataset.int_list[i], int_stiffness);
                // Add the intensity cost function to the problem
                problem.AddResidualBlock(
                    intensity_cost, NULL,
                    dataset.pose_vectors[k].data(),
                    dataset.map_vertices[j].position().data(),
                    dataset.map_vertices[j].normal().data(),
                    dataset.map_vertices[j].material()->phong_params().data(),
                    dataset.light_pos.data());
                // DEBUG: Hold normals constant
                problem.SetParameterBlockConstant(
                    dataset.map_vertices[j].normal().data());
            }
        }
    }

    // DEBUG: Hold light position constant
    // problem.SetParameterBlockConstant(dataset.light_pos.data());

    // Hold the first pose constant
    problem.SetParameterBlockConstant(dataset.pose_vectors[0].data());

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
