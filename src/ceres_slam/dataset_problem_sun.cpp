#include <ceres_slam/dataset_problem_sun.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/point_cloud_aligner.hpp>
#include <ceres_slam/stereo_camera.hpp>
#include <ceres_slam/utils/csv_reader.hpp>

namespace ceres_slam {

const bool DatasetProblemSun::read_csv(const std::string track_file,
                                       const std::string ref_sun_file,
                                       const std::string obs_sun_file) {
    //////////////////////////////////////////////////////////////////////////
    // Read stereo observations and metadata
    std::cerr << "Loading file " << track_file << std::endl;
    CSVReader reader(track_file);

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
    pose_covars.resize(num_states);
    map_points.resize(num_points);
    initialized_point.resize(num_points);
    reset_points();

    sun_dir_g.resize(num_states);
    sun_obs_list.resize(num_states);
    sun_obs_covars.resize(num_states);
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

    // Set first ground truth pose covariance to something small
    std::cerr << "Setting first ground truth pose covariance" << std::endl;
    pose_covars[0] = 1e-12 * SE3::AdjointMatrix::Identity();
    std::cout << "Initial covariance:\n" << pose_covars[0] << std::endl;

    // Read in the observations
    std::cerr << "Reading observation data... ";
    while (!reader.atEOF()) {
        tokens = reader.getLine();
        uint k = std::stoi(tokens.at(0));
        state_ids.push_back(k);
        point_ids.push_back(std::stoi(tokens.at(1)));
        double u = std::stod(tokens.at(2));
        double v = std::stod(tokens.at(3));
        double d = std::stod(tokens.at(4));
        Camera::ObservationCovariance uvd_covar;
        uvd_covar << std::stod(tokens.at(5)), std::stod(tokens.at(6)),
            std::stod(tokens.at(7)), std::stod(tokens.at(8)),
            std::stod(tokens.at(9)), std::stod(tokens.at(10)),
            std::stod(tokens.at(11)), std::stod(tokens.at(12)),
            std::stod(tokens.at(13));

        stereo_obs_list.push_back(Camera::Observation(u, v, d));
        stereo_obs_covars.push_back(uvd_covar);
    }

    std::cerr << "read " << stereo_obs_list.size() << " stereo observations"
              << std::endl;
    std::cout << "First stereo measurment covariance:\n"
              << stereo_obs_covars[0] << std::endl;

    // Generate lists of observation indices for each state and feature
    std::cerr
        << "Generating observation indices for each state and feature...\n";
    state_indices_.resize(num_states);
    feature_indices_.resize(num_points);

    for (uint idx = 0; idx < state_ids.size(); ++idx) {
        uint k = state_ids.at(idx);
        state_indices_.at(k).push_back(idx);

        uint j = point_ids.at(idx);
        feature_indices_.at(j).push_back(idx);
    }

    //////////////////////////////////////////////////////////////////////////
    // Read ephemeris sun vectors in global ENU coordinates
    std::cerr << "Reading sun ephemeris data (in ENU coordinates)...\n";
    std::cerr << "Loading file " << ref_sun_file << std::endl;
    CSVReader reader2(ref_sun_file);

    std::vector<std::string> tokens2;

    // Quit if you can't open the file
    if (!reader2.isOpen()) {
        return false;
    }

    while (!reader2.atEOF()) {
        tokens2 = reader2.getLine();
        uint k = stoi(tokens2.at(0));
        double e = stod(tokens2.at(1));
        double n = stod(tokens2.at(2));
        double u = stod(tokens2.at(3));
        sun_dir_g.at(k) << e, n, u;
    }

    //////////////////////////////////////////////////////////////////////////
    // Read sun observations in camera coordinates
    std::cerr << "Reading sun observations (in camera coordinates)...\n";
    std::cerr << "Loading file " << obs_sun_file << std::endl;
    CSVReader reader3(obs_sun_file);

    std::vector<std::string> tokens3;

    // Quit if you can't open the file
    if (!reader3.isOpen()) {
        return false;
    }

    while (!reader3.atEOF()) {
        tokens3 = reader3.getLine();
        uint k = stoi(tokens3.at(0));
        double x = stod(tokens3.at(1));
        double y = stod(tokens3.at(2));
        double z = stod(tokens3.at(3));
        SunCovariance az_zen_covar;
        az_zen_covar << stod(tokens3.at(4)), stod(tokens3.at(5)),
            stod(tokens3.at(6)), stod(tokens3.at(7));

        sun_obs_list.at(k) << x, y, z;
        sun_obs_covars.at(k) = az_zen_covar;
        state_has_sun_obs.at(k) = true;
    }

    std::cout << "First sun measurment covariance:\n"
              << sun_obs_covars[0] << std::endl;
    std::cerr << "done." << std::endl;

    return true;
}

const bool DatasetProblemSun::write_csv(std::string filename) const {
    std::vector<std::string> tokens = split(filename, '.');
    std::string filename_poses = tokens.at(0) + "_poses.csv";
    // std::string filename_map = tokens.at(0) + "_map.csv";

    // std::cout << "Outputting to files:"
    //           << "\n\t" << filename_poses << "\n\t" << filename_map
    //           << std::endl;
    std::cout << "Outputting to file:"
              << "\n\t" << filename_poses << std::endl;

    // Open files
    std::ofstream pose_file(filename_poses);
    // std::ofstream map_file(filename_map);

    // Quit if you can't open the files
    if (!pose_file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename_poses
                  << std::endl;
        return false;
    }
    // if (!map_file.is_open()) {
    //     std::cerr << "Error: Couldn't open file " << filename_map <<
    //     std::endl;
    //     return false;
    // }

    // Convert poses to CSV entries
    pose_file << "T_00, T_01, T_02, T_03,"
              << "T_10, T_11, T_12, T_13,"
              << "T_20, T_21, T_22, T_23,"
              << "T_30, T_31, T_32, T_33" << std::endl;
    for (SE3 T : poses) {
        pose_file << T.str() << std::endl;
    }

    // Convert initialized points to CSV entries
    // map_file << "point_id, x, y, z" << std::endl;
    // for (uint j = 0; j < map_points.size(); ++j) {
    //     if (initialized_point[j]) {
    //         map_file << j << "," << map_points[j].str() << std::endl;
    //     }
    // }

    // Close files
    pose_file.close();
    // map_file.close();

    return true;
}

const std::vector<uint> DatasetProblemSun::obs_indices_at_state(uint k) const {
    return state_indices_.at(k);
}

const std::vector<uint> DatasetProblemSun::obs_indices_for_feature(
    uint j) const {
    return feature_indices_.at(j);
}

void DatasetProblemSun::reset_points() {
    std::fill(initialized_point.begin(), initialized_point.end(), false);
}

bool DatasetProblemSun::compute_initial_guess(uint k1, uint k2) {
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
        // std::cout << "k = " << k << ", k-1 = " << k - 1 << std::endl;
        // std::cout << "Initial set has " << pts_km1.size() << " elements"
        //           << std::endl;

        SE3 T_k_km1;
        std::vector<uint> inlier_idx =
            point_cloud_aligner.compute_transformation_and_inliers(
                T_k_km1, pts_km1, pts_k, camera, 400, 4.);

        // std::cout << "Best inlier set has " << inlier_idx.size() << "
        // elements."
        //           << std::endl;
        // std::cout << "T_1_0 = " << std::endl << T_k_km1 << std::endl;

        if (inlier_idx.size() < 3) {
            std::cout << "WARNING: Fewer than 3 inliers found." << std::endl;
            return false;
        }

        // Compound the transformation estimate onto the previous one
        poses[k] = T_k_km1 * poses[k - 1];

        // If the map point does not have an initial guess already,
        // initialize it
        for (uint i : inlier_idx) {
            if (!initialized_point[j_km1[i]]) {
                // Initialize the position with the guess from the
                // first point cloud, transformed into the base frame
                Point point_position = poses[k - 1].inverse() * pts_km1[i];
                // std::cout << "k-1: " << k - 1 << "\tj: " << j_km1[i]
                //           << "\n T_w_km1:\n"
                //           << poses[k - 1].inverse()
                //           << "\n pt_km1: " << pts_km1[i]
                //           << "\n point_g: " << point_position << std::endl;

                // Create the point object
                map_points[j_km1[i]] = point_position;

                // Set the initialization flag to true for this point
                initialized_point[j_km1[i]] = true;
            }
        }
    }

    return true;
}

}  // namespace ceres_slaml
