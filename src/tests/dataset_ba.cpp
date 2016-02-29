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
                << "[--nolight | --dirlight] [--multistage]"
                << std::endl;
      return EXIT_FAILURE;
    }

    // Defaults
    bool use_light = true;
    bool directional_light = false;
    bool multi_stage = false;

    // Parse command line arguments
    std::string filename(argv[1]);
    for(int a = 2; a < argc; ++a) {
        std::string flag(argv[a]);

        if(flag == "--nolight") {
            use_light = false;
            directional_light = false;
        }
        else if(flag == "--dirlight") {
            use_light = true;
            directional_light = true;
        }
        else if(flag == "--multistage") {
            multi_stage = true;
            use_light = true;
        }
        else {
            std::cerr << "usage: dataset_ba <input_file> "
                      << "[--nolight | --dirlight] [--multistage]"
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
            }
        }

        // Use local parameterization on SE(3)
        problem.SetParameterization(dataset.poses[k].data(), se3_perturbation);
    }

    // Hold the first pose constant
    problem.SetParameterBlockConstant(dataset.poses[0].data() );

    // Set solver options
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.num_threads = 8;
    solver_options.num_linear_solver_threads = 8;
    solver_options.max_num_iterations = 1000;
    solver_options.use_nonmonotonic_steps = true;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // solver_options.check_gradients = true;

    // Create sumary container
    ceres::Solver::Summary summary;

    // Solve!
    ///////////////////////////////////////////////////////////////////////
    // Stage 1 - Optimize points and poses only, no lighting
    if(multi_stage) {
        std::cerr << "Solving stage 1: poses and points" << std::endl;
        Solve(solver_options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
    }

    // Add the lighting terms to the problem
    if(use_light) {
        for(unsigned int k = 0; k < dataset.num_states; ++k) {
            for(unsigned int i : dataset.obs_indices_at_state(k) ) {
                // Map point ID for this observation
                unsigned int j = dataset.vertex_ids[i];

                if(dataset.initialized_vertex[j]) {
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
                            ->phong_params().data(), 2, 1.);

                    // DEBUG: Hold material parameters constant
                    // problem.SetParameterBlockConstant(dataset.map_vertices[j]
                    //     .material()->phong_params().data() );

                    // DEBUG: Hold texture constant
                    // problem.SetParameterBlockConstant(dataset.map_vertices[j]
                    //     .texture_ptr() );

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

        // Constrain light direction updates to stay on unit sphere
        if(directional_light) {
            problem.SetParameterization(dataset.light_dir.data(),
                                        unit_vector_perturbation);
        }

        // DEBUG: Hold light position constant
        // problem.SetParameterBlockConstant(dataset.light_pos.data() );
    }

    ///////////////////////////////////////////////////////////////////////
    // Stage 2 - Optimize lighting only, hold poses and points constant
    if(multi_stage) {
        for(unsigned int k = 0; k < dataset.num_states; ++k) {
            for(unsigned int i : dataset.obs_indices_at_state(k) ) {
                // Map point ID for this observation
                unsigned int j = dataset.vertex_ids[i];

                if(dataset.initialized_vertex[j] ) {
                    problem.SetParameterBlockConstant(
                        dataset.map_vertices[j].position().data() );
                }
            }

            problem.SetParameterBlockConstant(dataset.poses[k].data() );
        }

        std::cerr << "Solving stage 2: lighting" << std::endl;
        Solve(solver_options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;

        for(unsigned int k = 0; k < dataset.num_states; ++k) {
            for(unsigned int i : dataset.obs_indices_at_state(k) ) {
                // Map point ID for this observation
                unsigned int j = dataset.vertex_ids[i];

                if(dataset.initialized_vertex[j] ) {
                    problem.SetParameterBlockVariable(
                        dataset.map_vertices[j].position().data() );
                }
            }

            if(k > 0) {
                problem.SetParameterBlockVariable(dataset.poses[k].data() );
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    // Stage 3 / Default option - Optimize lighting, poses, and points jointly
    std::cerr << "Solving SLAM and lighting jointly" << std::endl;
    Solve(solver_options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // Estimate covariance?

    // Output optimized state to file
    std::cerr << "Outputting to file " << std::endl;
    dataset.write_csv(filename);

    return EXIT_SUCCESS;
}
