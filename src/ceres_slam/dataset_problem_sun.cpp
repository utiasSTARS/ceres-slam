#include <ceres_slam/dataset_problem_sun.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <string>

#include <ceres_slam/utils/csv_reader.h>
#include <ceres_slam/geometry/geometry.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/point_cloud_aligner.h>

namespace ceres_slam {

const bool DatasetProblemSun::read_csv(std::string filename) {
    std::cerr << "Loading file " << filename << std::endl;
    CSVReader reader(filename);

    std::vector<std::string> tokens;

    // Quit if you can't open the file
    if (!reader.isOpen()) {
        return false;
    }

    // Read number of states and points and allocate space
    std::cerr << "Reading metadata... ";
    tokens = reader.getLine();
    num_states = std::stoi(tokens.at(0));
    num_points = std::stoi(tokens.at(1));
    poses.resize(num_states);
    map_points.resize(num_points);
    initialized_point.resize(num_points);
    reset_points();

    sun_obs_list.resize(num_states);
    for (uint k = 0; k < num_states; ++k) {
        state_has_sun_obs.push_back(false);
    }

    std::cerr << "expecting " << num_states << " states"
              << " and " << num_points << " points" << std::endl;

    // Read camera intrinsics
    std::cerr << "Reading camera intrinsics" << std::endl;
    tokens = reader.getLine();
    double fu = std::stod(tokens.at(0));
    double fv = std::stod(tokens.at(1));
    double cu = std::stod(tokens.at(2));
    double cv = std::stod(tokens.at(3));
    double b = std::stod(tokens.at(4));
    camera = std::make_shared<Camera>(fu, fv, cu, cv, b);
    std::cerr << *camera << std::endl;

    // Read observation variance
    std::cerr << "Reading observation variances" << std::endl;
    tokens = reader.getLine();
    stereo_obs_var << std::stod(tokens.at(0)), std::stod(tokens.at(1)),
        std::stod(tokens.at(2));
    sun_obs_var << std::stod(tokens.at(3)), std::stod(tokens.at(4)),
        std::stod(tokens.at(5));

    // stereo_obs_var *= 1e2;
    // sun_obs_var *= 3.;

    std::cerr << "Stereo observation variance: " << stereo_obs_var.transpose()
              << std::endl
              << "Sun direction observation variance: "
              << sun_obs_var.transpose() << std::endl;

    // Read first ground truth pose
    std::cerr << "Reading first ground truth pose" << std::endl;
    tokens = reader.getLine();
    SE3::TransformationMatrix T_0_g;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_0_g(i, j) = std::stod(tokens.at(4 * i + j));
        }
    }
    // First pose is either identity, or the first ground truth pose
    poses[0] = SE3(T_0_g);
    std::cout << poses[0] << std::endl;

    // Read ground truth sun direction in the global frame
    std::cerr << "Reading ground truth sun direction... " << std::endl;
    tokens = reader.getLine();
    sun_dir_g << std::stod(tokens.at(0)), std::stod(tokens.at(1)),
        std::stod(tokens.at(2));
    std::cerr << sun_dir_g << std::endl;

    // Read in the observations
    std::cerr << "Reading observation data... ";
    while (!reader.atEOF()) {
        tokens = reader.getLine();
        uint k = std::stod(tokens.at(0));
        if (tokens.size() > 4) {
            // This is a stereo observation
            state_ids.push_back(k);
            point_ids.push_back(std::stoi(tokens.at(1)));
            double u = std::stod(tokens.at(2));
            double v = std::stod(tokens.at(3));
            double d = std::stod(tokens.at(4));
            stereo_obs_list.push_back(Camera::Observation(u, v, d));
        } else {
            // This is a sun direction observation
            state_has_sun_obs.at(k) = true;
            sun_obs_list.at(k) << std::stod(tokens.at(1)),
                std::stod(tokens.at(2)), std::stod(tokens.at(3));
        }
    }
    std::cerr << "read " << stereo_obs_list.size() << " stereo observations"
              << std::endl;

    // Generate a list of observation indices for each state
    std::cerr << "Generating observation indices for each state... ";
    std::vector<uint> k_indices;
    k_indices.push_back(0);
    for (uint idx = 1; idx < state_ids.size(); ++idx) {
        if (state_ids.at(idx) != state_ids.at(idx - 1)) {
            state_indices_.push_back(k_indices);
            k_indices.clear();
        }
        k_indices.push_back(idx);
    }
    state_indices_.push_back(k_indices);
    std::cerr << "found " << state_indices_.size() << " unique states"
              << std::endl;

    // Generate a list of observation indices for each feature
    std::cerr << "Generating observation indices for each feature... ";
    feature_indices_.resize(num_points);
    std::vector<uint> j_indices;

    for (uint j = 0; j < num_points; ++j) {
        j_indices.clear();
        for (uint idx = 0; idx < point_ids.size(); idx++) {
            if (point_ids.at(idx) == j) {
                j_indices.push_back(idx);
            }
        }
        feature_indices_.at(j) = j_indices;
    }

    std::cerr << "found " << feature_indices_.size() << " unique features"
              << std::endl;

    return true;
}

const bool DatasetProblemSun::write_csv(std::string filename) const {
    std::vector<std::string> tokens = split(filename, '.');
    std::string filename_poses = tokens.at(0) + "_poses.csv";
    std::string filename_map = tokens.at(0) + "_map.csv";

    // Open files
    std::ofstream pose_file(filename_poses);
    std::ofstream map_file(filename_map);

    // Quit if you can't open the files
    if (!pose_file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename_poses
                  << std::endl;
        return false;
    }
    if (!map_file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename_map << std::endl;
        return false;
    }

    // Convert poses to CSV entries
    pose_file << "T_00, T_01, T_02, T_03,"
              << "T_10, T_11, T_12, T_13,"
              << "T_20, T_21, T_22, T_23,"
              << "T_30, T_31, T_32, T_33" << std::endl;
    for (SE3 T : poses) {
        pose_file << T.str() << std::endl;
    }

    // Convert initialized points to CSV entries
    map_file << "point_id, x, y, z" << std::endl;
    for (uint j = 0; j < map_points.size(); ++j) {
        if (initialized_point[j]) {
            map_file << j << "," << map_points[j].str() << std::endl;
        }
    }

    // Close files
    pose_file.close();
    map_file.close();

    return true;
}

const std::vector<uint> DatasetProblemSun::obs_indices_at_state(uint k) const {
    return state_indices_.at(k);
}

const std::vector<uint> DatasetProblemSun::obs_indices_for_feature(uint j) const {
    return feature_indices_.at(j);
}

void DatasetProblemSun::reset_points() {
    std::fill(initialized_point.begin(), initialized_point.end(), false);
}

void DatasetProblemSun::compute_initial_guess(uint k1, uint k2) {
    std::vector<Point> pts_km1, pts_k;
    std::vector<uint> j_km1, j_k, idx_km1, idx_k;
    ceres_slam::PointCloudAligner point_cloud_aligner;

    if (k1 >= k2) {
        k1 = 0;
        k2 = num_states;
    }

    // Iterate over all poses
    for (uint k = k1 + 1; k < k2; ++k) {
        // std::cout << "k = " << k << std::endl;

        pts_km1.clear();
        pts_k.clear();
        j_km1.clear();
        j_k.clear();
        idx_km1 = obs_indices_at_state(k - 1);
        idx_k = obs_indices_at_state(k);

        // Find point IDs for both poses
        for (uint i : idx_km1) {
            j_km1.push_back(point_ids[i]);
        }
        for (uint i : idx_k) {
            j_k.push_back(point_ids[i]);
        }

        // Find reciprocal point matches and delete unmatched indices
        for (uint i = 0; i < j_km1.size(); ++i) {
            if (std::find(j_k.begin(), j_k.end(), j_km1[i]) == j_k.end()) {
                j_km1.erase(j_km1.begin() + i);
                idx_km1.erase(idx_km1.begin() + i);
                --i;
            }
        }
        for (uint i = 0; i < j_k.size(); ++i) {
            if (std::find(j_km1.begin(), j_km1.end(), j_k[i]) == j_km1.end()) {
                j_k.erase(j_k.begin() + i);
                idx_k.erase(idx_k.begin() + i);
                --i;
            }
        }

        // Triangulate map points from each pose
        for (uint i : idx_km1) {
            pts_km1.push_back(camera->triangulate(stereo_obs_list[i]));
        }
        for (uint i : idx_k) {
            pts_k.push_back(camera->triangulate(stereo_obs_list[i]));
        }

        // for(uint i = 0; i < j_km1.size(); ++i) {
        //     std::cout << "j_km1: " << j_km1[i] << " | j_k: " << j_k[i]
        //               << " | idx_km1: " << idx_km1[i] << " | idx_k: " <<
        //               idx_k[i]
        //               << " | pts_k - pts_km1: " << pts_k[i] - pts_km1[i]
        //               << std::endl;
        // }

        // Compute the transform from the first to the second point cloud
        // std::cout << "k = " << k << ", k-1 = " << k-1 << std::endl;
        // std::cout <<"Initial set has " << pts_km1.size()
        //           << " elements" << std::endl;

        SE3 T_k_km1;
        std::vector<uint> inlier_idx =
            point_cloud_aligner.compute_transformation_and_inliers(
                T_k_km1, pts_km1, pts_k, camera, 400, 4.);

        // std::cout <<"Best inlier set has " << inlier_idx.size()
        //               << " elements" << std::endl;
        // std::cout << "T_1_0 = " << std::endl << T_k_km1 << std::endl;

        // Compound the transformation estimate onto the previous one
        poses[k] = T_k_km1 * poses[k - 1];

        // If the map point does not have an initial guess already,
        // initialize it
        for (uint i : inlier_idx) {
            if (!initialized_point[j_km1[i]]) {
                // Initialize the position with the guess from the
                // first point cloud, transformed into the base frame
                Point point_position = poses[k - 1].inverse() * pts_km1[i];

                // Create the point object
                map_points[j_km1[i]] = point_position;

                // Set the initialization flag to true for this point
                initialized_point[j_km1[i]] = true;
            }
        }
    }
}

}  // namespace ceres_slam
