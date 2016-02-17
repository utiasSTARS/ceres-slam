#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>

#include <ceres/ceres.h>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry.h>
#include <ceres_slam/lighting.h>
#include <ceres_slam/dataset_problem.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/stereo_reprojection_error.h>
#include <ceres_slam/intensity_error_point_light.h>
#include <ceres_slam/intensity_error_directional_light.h>
#include <ceres_slam/normal_error.h>
#include <ceres_slam/perturbations.h>

#include <Eigen/Eigenvalues>

using SE3 = ceres_slam::DatasetProblem::SE3;
using Point = ceres_slam::DatasetProblem::Point;
using Vector = ceres_slam::DatasetProblem::Vector;
using Camera = ceres_slam::DatasetProblem::Camera;

int main(int argc, char** argv) {
    if(argc < 2) {
      std::cerr << "usage: dataset_ba <input_file> "
                << "[--nolight | --dirlight]"
                << std::endl;
      return EXIT_FAILURE;
    }

    // Defaults
    bool use_light = true;
    bool directional_light = false;

    // Parse command line arguments
    std::string filename(argv[1]);
    if(argc >= 3) {
        std::string flag(argv[2]);
        if(flag == "--nolight") {
            use_light = false;
        }
        else if(flag == "--dirlight") {
            use_light = true;
            directional_light = true;
        }
        else {
            std::cerr << "usage: dataset_ba <input_file> "
                      << "[--nolight/--dirlight]"
                      << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Read dataset from file
    ceres_slam::DatasetProblem dataset(directional_light);
    if(!dataset.read_csv(filename) ) {
        return EXIT_FAILURE;
    }

    // Compute initial guess
    std::cerr << "Computing VO initial guess" << std::endl;
    dataset.compute_initial_guess();

    // Output the initial guess to a CSV file for comparison
    std::vector<std::string> tokens;
    tokens = ceres_slam::split(filename, '.');
    dataset.write_csv(tokens.at(0) + "_initial.csv");

    // Build the problem
    std::cerr << "Building problem" << std::endl;
    ceres::Problem problem;

    dataset.stereo_obs_var << 4., 4., 16.; // u,v,d variance
    dataset.normal_obs_var << 0.01, 0.01, 0.01; // i,j,k variance
    dataset.int_var = 0.01; // I variance

    // Compute the stiffness matrix to apply to the residuals
    Eigen::SelfAdjointEigenSolver<Camera::ObservationCovariance>
        es_stereo(dataset.stereo_obs_var.asDiagonal() );
    Camera::ObservationCovariance stereo_obs_stiffness =
        es_stereo.operatorInverseSqrt();

    Eigen::SelfAdjointEigenSolver<Vector::Covariance>
        es_normal(dataset.normal_obs_var.asDiagonal() );
    Camera::ObservationCovariance normal_obs_stiffness =
        es_normal.operatorInverseSqrt();

    double int_stiffness = 1. / sqrt(dataset.int_var);

    // Set up local parameterizations
    ceres::LocalParameterization* se3_perturbation
        = ceres_slam::SE3Perturbation::Create();
    ceres::LocalParameterization* unit_vector_perturbation
        = ceres_slam::UnitVectorPerturbation::Create();

    // Add observations and cost functions
    for(unsigned int k = 0; k < dataset.num_states; ++k) {
        for(unsigned int i : dataset.obs_indices_at_state(k) ) {
            // Map point ID for this observation
            unsigned int j = dataset.vertex_ids[i];
            // Only optimize map points that have been initialized
            if(dataset.initialized_vertex[j] ) {
                // Cost function for the stereo observation
                ceres::CostFunction* stereo_cost =
                    ceres_slam::StereoReprojectionErrorAutomatic::Create(
                        dataset.camera,
                        dataset.stereo_obs_list[i],
                        stereo_obs_stiffness);
                // Add the stereo cost function to the problem
                problem.AddResidualBlock(
                    stereo_cost, NULL,
                    dataset.poses[k].data(),
                    dataset.map_vertices[j].position().data() );

                if(use_light) {
                    if(directional_light) {
                        ceres::CostFunction* intensity_cost =
                            ceres_slam::IntensityErrorDirectionalLightAutomatic::Create(
                                dataset.int_list[i],
                                int_stiffness);
                        // Add the intensity cost function to the problem
                        problem.AddResidualBlock(
                            intensity_cost, NULL,
                            dataset.poses[k].data(),
                            dataset.map_vertices[j].position().data(),
                            dataset.map_vertices[j].normal().data(),
                            dataset.map_vertices[j].material()
                                ->phong_params().data(),
                            dataset.map_vertices[j].texture_ptr(),
                            dataset.light_dir.data() );
                    } else {
                        // Cost function for the intensity observation
                        ceres::CostFunction* intensity_cost =
                            ceres_slam::IntensityErrorPointLightAutomatic::Create(
                                dataset.int_list[i],
                                int_stiffness);
                        // Add the intensity cost function to the problem
                        problem.AddResidualBlock(
                            intensity_cost, NULL,
                            dataset.poses[k].data(),
                            dataset.map_vertices[j].position().data(),
                            dataset.map_vertices[j].normal().data(),
                            dataset.map_vertices[j].material()
                                ->phong_params().data(),
                            dataset.map_vertices[j].texture_ptr(),
                            dataset.light_pos.data() );
                    }

                    // Set upper and lower bounds on Phong parameters
                    problem.SetParameterLowerBound(
                        dataset.map_vertices[j].material()
                            ->phong_params().data(), 0, 0.);
                    problem.SetParameterUpperBound(
                        dataset.map_vertices[j].material()
                            ->phong_params().data(), 0, 1.);
                    problem.SetParameterLowerBound(
                        dataset.map_vertices[j].material()
                            ->phong_params().data(), 1, 0.);
                    problem.SetParameterUpperBound(
                        dataset.map_vertices[j].material()
                            ->phong_params().data(), 1, 1.);
                    problem.SetParameterLowerBound(
                        dataset.map_vertices[j].material()
                            ->phong_params().data(), 2, 0.);

                    // Set upper and lower bounds on texture values
                    problem.SetParameterLowerBound(
                        dataset.map_vertices[j].texture_ptr(), 0, 0.);
                    problem.SetParameterUpperBound(
                        dataset.map_vertices[j].texture_ptr(), 0, 1.);

                    // Cost function for the normal observation
                    ceres::CostFunction* normal_cost =
                        ceres_slam::NormalErrorAutomatic::Create(
                            dataset.normal_obs_list[i],
                            normal_obs_stiffness);
                    // Add the normal cost function to the problem
                    problem.AddResidualBlock(
                        normal_cost, NULL,
                        dataset.poses[k].data(),
                        dataset.map_vertices[j].normal().data() );

                    // Constrain normal vector updates to stay on unit sphere
                    problem.SetParameterization(
                        dataset.map_vertices[j].normal().data(),
                        unit_vector_perturbation);
                }
            }
        }

        // Use local parameterization on SE(3)
        problem.SetParameterization(dataset.poses[k].data(), se3_perturbation);

        // Constrain light direction updates to stay on unit sphere
        if(use_light && directional_light) {
            problem.SetParameterization(dataset.light_dir.data(),
                                        unit_vector_perturbation);
        }
    }

    // DEBUG: Hold light position constant
    // problem.SetParameterBlockConstant(dataset.light_pos.data() );

    // Hold the first pose constant
    problem.SetParameterBlockConstant(dataset.poses[0].data() );

    // Solve the problem
    std::cerr << "Solving" << std::endl;
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.num_threads = 8;
    solver_options.num_linear_solver_threads = 8;
    solver_options.max_num_iterations = 1000;

    // solver_options.trust_region_strategy_type = ceres::DOGLEG;
    // solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
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
