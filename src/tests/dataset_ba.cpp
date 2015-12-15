#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres/ceres.h>

#include <ceres_slam/dataset_problem.h>
#include <ceres_slam/point_cloud_aligner.h>
#include <ceres_slam/stereo_reprojection_error.h>

#include <Eigen/Eigenvalues>

using SE3 = ceres_slam::DatasetProblem::SE3;
using Point = ceres_slam::DatasetProblem::Point;
using Camera = ceres_slam::DatasetProblem::Camera;

int main(int argc, char** argv) {
    if (argc < 2) {
      std::cerr << "usage: dataset_ba <input_file> <optional: output_file>"
                << std::endl;
      return EXIT_FAILURE;
    }

    // Read dataset from file
    ceres_slam::DatasetProblem dataset;
    if( !dataset.read_csv(std::string(argv[1])) ) {
        return EXIT_FAILURE;
    }

    // Generate initial guess for poses and map points
    // using scalar-weighted point cloud alignment for stereo VO
    std::vector<Point> pts_km1, pts_k;
    std::vector<unsigned int> j_km1, j_k, idx_km1, idx_k;
    ceres_slam::PointCloudAligner point_cloud_aligner;

    // First pose is either identity, or the first ground truth pose
    dataset.pose_vectors[0] = SE3::log(dataset.first_pose);

    // Iterate over all poses
    std::cerr << "Computing VO initial guess" << std::endl;
    for(unsigned int k = 1; k < dataset.num_states; ++k) {
        pts_km1.clear();
        pts_k.clear();
        j_km1.clear();
        j_k.clear();
        idx_km1 = dataset.obs_indices_at_state(k-1);
        idx_k = dataset.obs_indices_at_state(k);

        // Find point IDs for both poses
        for(unsigned int i : idx_km1) { j_km1.push_back(dataset.point_ids[i]); }
        for(unsigned int i : idx_k) { j_k.push_back(dataset.point_ids[i]); }

        // Find reciprocal point matches and delete unmatched indices
        for(unsigned int i = 0; i < j_km1.size(); ++i) {
            if(std::find(j_k.begin(), j_k.end(), j_km1[i]) == j_k.end()) {
                j_km1.erase(j_km1.begin() + i);
                idx_km1.erase(idx_km1.begin() + i);
                --i;
            }
        }
        for(unsigned int i = 0; i < j_k.size(); ++i) {
            if(std::find(j_km1.begin(), j_km1.end(), j_k[i]) == j_km1.end()) {
                j_k.erase(j_k.begin() + i);
                idx_k.erase(idx_k.begin() + i);
                --i;
            }
        }

        // Triangulate map points from each pose
        for(unsigned int i : idx_km1) {
            pts_km1.push_back(dataset.camera->triangulate(dataset.obs_list[i]));
        }
        for(unsigned int i : idx_k) {
            pts_k.push_back(dataset.camera->triangulate(dataset.obs_list[i]));
        }

        // for(unsigned int i = 0; i < j_km1.size(); ++i) {
        //     std::cout << "j_km1: " << j_km1[i] << " | j_k: " << j_k[i]
        //               << " | idx_km1: " << idx_km1[i] << " | idx_k: " << idx_k[i]
        //               << " | pts_k - pts_km1: " << pts_k[i] - pts_km1[i]
        //               << std::endl;
        // }

        // Compute the transform from the first to the second point cloud
        // std::cout <<"Initial set has " << pts_km1.size()
        //           << " elements" << std::endl;

        SE3 T_k_km1 = point_cloud_aligner.compute_transformation_and_inliers(
            pts_km1, pts_k, dataset.camera, 400, 9);

        // std::cout <<"Best inlier set has " << pts_km1.size()
        //               << " elements" << std::endl;
        // std::cout << "T_1_0 = " << std::endl << T_k_km1 << std::endl;

        // Compound the transformation estimate onto the previous one
        dataset.pose_vectors[k] =
            SE3::log(T_k_km1 * SE3::exp(dataset.pose_vectors[k-1]));

        // If the map point does not have an initial guess already, use the
        // guess from the first point cloud, transformed into the base frame
        for(unsigned int i = 0; i < j_km1.size(); ++i) {
            if(!dataset.initialized_point[ j_km1[i] ]) {
                dataset.map_points[ j_km1[i] ] =
                    SE3::exp(dataset.pose_vectors[k-1]).inverse() * pts_km1[i];
                dataset.initialized_point[ j_km1[i] ] = true;
            }
        }
    }

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
    // solver_options.linear_solver_type = ceres::DENSE_SCHUR;
    // solver_options.check_gradients = true;

    ceres::Solver::Summary summary;
    Solve(solver_options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    // Estimate covariance?

    // Output optimized state to file
    std::string out_file("optimized.csv");
    if (argc > 2) {
        out_file = std::string(argv[2]);
    }
    std::cerr << "Outputting to file " << out_file << std::endl;
    dataset.write_csv(out_file);

    return EXIT_SUCCESS;
}
