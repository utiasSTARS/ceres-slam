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

    // Generate initial guess for poses and map points
    // using scalar-weighted point cloud alignment for stereo VO
    std::vector<ceres_slam::DatasetProblem::Point> pts_km1, pts_k;
    std::vector<unsigned int> j_km1, j_k, idx_km1, idx_k;
    ceres_slam::PointCloudAligner point_cloud_aligner;

    // First pose is the base frame
    dataset.poses[0] = ceres_slam::DatasetProblem::SE3(); // Identity

    // Iterate over all poses
    for(unsigned int k = 1; k < dataset.num_states; ++k) {
        pts_km1.clear();
        pts_k.clear();
        j_km1.clear();
        j_k.clear();
        idx_km1 = dataset.indices_at_state(k-1);
        idx_k = dataset.indices_at_state(k);

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

        // Compute the transform from the first to the second point cloud
        ceres_slam::DatasetProblem::SE3 T_k_km1 =
            point_cloud_aligner.compute_transformation(&pts_km1, &pts_k);

        // Compound the transformation estimate onto the previous one
        dataset.poses[k] = T_k_km1 * dataset.poses[k-1];

        // If the map point does not have an initial guess already, use the
        // guess from the first point cloud, transformed into the base frame
        for(unsigned int i = 0; i < j_km1.size(); ++i) {
            if(!dataset.initialized_point[ j_km1[i] ]) {
                dataset.map_points[ j_km1[i] ] =
                    dataset.poses[k-1].inverse() * pts_km1[i];
                dataset.initialized_point[ j_km1[i] ] = true;
            }
        }
    }

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
    std::string out_file("optimized.csv");
    if (argc > 2) {
        out_file = std::string(argv[2]);
    }
    dataset.write_csv(out_file);

    return EXIT_SUCCESS;
}
