#include <ceres_slam/dataset_problem.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <string>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/point_cloud_aligner.h>

namespace ceres_slam {

const bool DatasetProblem::read_csv(std::string filename) {
    std::string line;
    std::vector<std::string> tokens;

    std::cerr << "Loading file " << filename << std::endl;
    std::ifstream file(filename);

    // Quit if you can't open the file
    if(!file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename << std::endl;
        return false;
    }

    // Read number of states and points and allocate space
    std::cerr << "Reading metadata... ";
    std::getline(file, line);
    tokens = split(line, ',');
    num_states = std::stoi(tokens.at(0));
    num_vertices = std::stoi(tokens.at(1));
    poses.resize(num_states);
    map_vertices.resize(num_vertices);
    for(unsigned int j = 0; j < num_vertices; ++j) {
        initialized_vertex.push_back(false);
    }

    std::cerr << "expecting " << num_states << " states"
              << " and " << num_vertices << " points" << std::endl;

    // Read camera intrinsics
    std::cerr << "Reading camera intrinsics" << std::endl;
    std::getline(file, line);
    tokens = split(line, ',');
    double fu = std::stod(tokens.at(0));
    double fv = std::stod(tokens.at(1));
    double cu = std::stod(tokens.at(2));
    double cv = std::stod(tokens.at(3));
    double b  = std::stod(tokens.at(4));
    camera = std::make_shared<Camera>(fu, fv, cu, cv, b);
    std::cerr << *camera << std::endl;

    // Read observation variance
    std::cerr << "Reading observation variances" << std::endl;
    std::getline(file, line);
    tokens = split(line, ',');
    stereo_obs_var << std::stod(tokens.at(0)),
                      std::stod(tokens.at(1)),
                      std::stod(tokens.at(2));
    normal_obs_var << std::stod(tokens.at(3)),
                      std::stod(tokens.at(4)),
                      std::stod(tokens.at(5));
    int_var = std::stod(tokens.at(6));
    std::cerr << "Stereo observation variance: "
              << stereo_obs_var.transpose() << std::endl;
    std::cerr << "Normal observation variance: "
              << normal_obs_var.transpose() << std::endl;
    std::cerr << "Intensity variance: " << int_var << std::endl;

    // Read the initial light position/direction
    // Need to find a better way of initializing this in the future
    if(directional_light) {
        std::cerr << "Reading initial light direction" << std::endl;
        std::getline(file, line);
        tokens = split(line, ',');
        initial_light_dir << std::stod(tokens.at(0)),
                             std::stod(tokens.at(1)),
                             std::stod(tokens.at(2));
        initial_light_dir.normalize();
        std::cerr << initial_light_dir << std::endl;
    } else {
        std::cerr << "Reading initial light position" << std::endl;
        std::getline(file, line);
        tokens = split(line, ',');
        initial_light_pos << std::stod(tokens.at(0)),
                             std::stod(tokens.at(1)),
                             std::stod(tokens.at(2));
        std::cerr << initial_light_pos << std::endl;
    }

    // Read first ground truth pose
    std::cerr << "Reading first ground truth pose" << std::endl;
    std::getline(file, line);
    tokens = split(line, ',');
    SE3::TransformationMatrix T_0_g;
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            T_0_g(i,j) = std::stod(tokens.at(4*i + j));
        }
    }
    first_pose = SE3(T_0_g);
    std::cout << first_pose << std::endl;

    // Read in the observations
    std::cerr << "Reading observation data... ";
    Vector normal_obs;
    while(std::getline(file, line)) {
        // std::cerr << line << std::endl;
        tokens = split(line, ',');
        t.push_back( std::stod(tokens.at(0)) );
        unsigned int j = std::stoi(tokens.at(1));
        vertex_ids.push_back(j);
        double u = std::stod(tokens.at(2));
        double v = std::stod(tokens.at(3));
        double d = std::stod(tokens.at(4));
        stereo_obs_list.push_back( Camera::Observation(u,v,d) );
        int_list.push_back( std::stod(tokens.at(5)) );
        normal_obs << std::stod(tokens.at(6)),
                      std::stod(tokens.at(7)),
                      std::stod(tokens.at(8));
        normal_obs_list.push_back(normal_obs);
    }
    std::cerr << "read " << stereo_obs_list.size() << " observations" << std::endl;

    // Generate a list of observation indices for each state
    std::cerr << "Generating observation indices from timestamps... ";
    std::vector<unsigned int> t_indices;
    t_indices.push_back(0);
    for(unsigned int idx = 1; idx < t.size(); ++idx) {
        if(t.at(idx) != t.at(idx-1)) {
            state_indices_.push_back(t_indices);
            t_indices.clear();
        }
        t_indices.push_back(idx);
    }
    state_indices_.push_back(t_indices);
    std::cerr << "found " << state_indices_.size()
              << " unique timestamps" << std::endl;

    // Generate a list of observation indices for each feature
    std::cerr << "Generating observation indices from feature IDs... ";
    feature_indices_.resize(num_vertices);
    std::vector<unsigned int> j_indices;

    for(unsigned int j = 0; j < num_vertices; ++j) {
        j_indices.clear();
        for(unsigned int idx = 0; idx < vertex_ids.size(); idx++) {
            if(vertex_ids.at(idx) == j) {
                j_indices.push_back(idx);
            }
        }
        feature_indices_.at(j) = j_indices;
    }

    std::cerr << "found " << feature_indices_.size()
            << " unique features" << std::endl;

    file.close();
    return true;
}

const bool DatasetProblem::write_csv(std::string filename) const {
    std::vector<std::string> tokens = split(filename, '.');
    std::string filename_poses = tokens.at(0) + "_poses.csv";
    std::string filename_map = tokens.at(0) + "_map.csv";
    std::string filename_lights = tokens.at(0) + "_lights.csv";

    // Open files
    std::ofstream pose_file(filename_poses);
    std::ofstream map_file(filename_map);
    std::ofstream light_file(filename_lights);

    // Quit if you can't open the files
    if(!pose_file.is_open()) {
        std::cerr << "Error: Couldn't open file "
                  << filename_poses << std::endl;
        return false;
    }
    if(!map_file.is_open()) {
        std::cerr << "Error: Couldn't open file "
                  << filename_map << std::endl;
        return false;
    }
    if(!light_file.is_open()) {
        std::cerr << "Error: Couldn't open file "
                  << filename_lights << std::endl;
        return false;
    }

    // Convert poses to CSV entries
    pose_file << "T_00, T_01, T_02, T_03,"
              << "T_10, T_11, T_12, T_13,"
              << "T_20, T_21, T_22, T_23,"
              << "T_30, T_31, T_32, T_33" << std::endl;
    for(SE3 T : poses) {
        pose_file << T.str() << std::endl;
    }

    // Convert initialized vertices to CSV entries
    map_file << "point_id, x, y, z, nx, ny, nz, ka, ks, exponent, kd"
             << std::endl;
    for(unsigned int j = 0; j < map_vertices.size(); ++j) {
        if(initialized_vertex[j]) {
            map_file << j << "," << map_vertices[j].str() << std::endl;
        }
    }

    // Convert light position/direction to CSV entries
    if(directional_light) {
        light_file << "i, j, k" << std::endl;
        light_file << light_dir.str() << std::endl;
    } else {
        light_file << "x, y, z" << std::endl;
        light_file << light_pos.str() << std::endl;
    }

    // Close files
    pose_file.close();
    map_file.close();
    light_file.close();

    return true;
}

const std::vector<unsigned int> DatasetProblem::obs_indices_at_state(int k) const {
    return state_indices_.at(k);
}

const std::vector<unsigned int> DatasetProblem::obs_indices_for_feature(int j) const {
    return feature_indices_.at(j);
}

void DatasetProblem::compute_initial_guess() {
    std::vector<Point> pts_km1, pts_k;
    std::vector<Vector> normals_km1;
    std::vector<double> intensities_km1;
    std::vector<unsigned int> j_km1, j_k, idx_km1, idx_k;
    ceres_slam::PointCloudAligner point_cloud_aligner;

    // First pose is either identity, or the first ground truth pose
    poses[0] = first_pose;

    // Initialize the material (assuming everything is the same material)
    material = std::make_shared< Material<double> >(
        Material<double>::PhongParams(0., 0., 10.) );

    // Iterate over all poses
    for(unsigned int k = 1; k < num_states; ++k) {
        pts_km1.clear();
        pts_k.clear();
        normals_km1.clear();
        intensities_km1.clear();
        j_km1.clear();
        j_k.clear();
        idx_km1 = obs_indices_at_state(k-1);
        idx_k = obs_indices_at_state(k);

        // Find point IDs for both poses
        for(unsigned int i : idx_km1) { j_km1.push_back(vertex_ids[i]); }
        for(unsigned int i : idx_k) { j_k.push_back(vertex_ids[i]); }

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
            pts_km1.push_back(camera->triangulate(stereo_obs_list[i]));
            // Also store the observed normals corresponding to each
            // point in case we need to use them as an initial guess
            normals_km1.push_back(normal_obs_list[i]);

            // Store the intensities of each observed point as well
            intensities_km1.push_back(int_list[i]);
        }
        for(unsigned int i : idx_k) {
            pts_k.push_back(camera->triangulate(stereo_obs_list[i]));
        }

        // for(unsigned int i = 0; i < j_km1.size(); ++i) {
        //     std::cout << "j_km1: " << j_km1[i] << " | j_k: " << j_k[i]
        //               << " | idx_km1: " << idx_km1[i] << " | idx_k: " << idx_k[i]
        //               << " | pts_k - pts_km1: " << pts_k[i] - pts_km1[i]
        //               << std::endl;
        // }

        // Compute the transform from the first to the second point cloud
        // std::cout << "k = " << k << ", k-1 = " << k-1 << std::endl;
        // std::cout <<"Initial set has " << pts_km1.size()
        //           << " elements" << std::endl;

        SE3 T_k_km1;
        std::vector<unsigned int> inlier_idx
            = point_cloud_aligner.compute_transformation_and_inliers(
                                    T_k_km1, pts_km1, pts_k, camera, 400, 9);

        // std::cout <<"Best inlier set has " << inlier_idx.size()
        //               << " elements" << std::endl;
        // std::cout << "T_1_0 = " << std::endl << T_k_km1 << std::endl;

        // Compound the transformation estimate onto the previous one
        poses[k] = T_k_km1 * poses[k-1];

        // Initialize the light source to something close to ground truth.
        // Need to figure out a way to do this in general.
        if(directional_light) {
            light_dir = initial_light_dir;
        } else {
            light_pos = initial_light_pos;
        }

        // If the map point does not have an initial guess already,
        // initialize it
        for(unsigned int i : inlier_idx) {
            if(!initialized_vertex[ j_km1[i] ]) {
                // Initialize the position with the guess from the
                // first point cloud, transformed into the base frame
                Point vertex_position = poses[k-1].inverse() * pts_km1[i];

                // Initialize the normal with the guess from the
                // first point cloud, transformed into the base frame
                Vector vertex_normal = poses[k-1].inverse() * normals_km1[i];

                // Create the vertex object
                map_vertices[ j_km1[i] ].position() = vertex_position;
                map_vertices[ j_km1[i] ].normal() = vertex_normal;
                map_vertices[ j_km1[i] ].material() = material;

                // Diffuse initialization
                // Option 1: use the first observed intensity
                // map_vertices[ j_km1[i] ].texture() = intensities_km1[i];
                // Option 2: use the minimum observed intensity
                // Option 3: use the median observed intensity
                std::vector<double> ints;
                for(unsigned int idx : obs_indices_for_feature(j_km1[i]) ) {
                    ints.push_back(int_list.at(idx));
                }
                std::nth_element(ints.begin(),
                                 ints.begin() + ints.size()/2, ints.end() );
                map_vertices[ j_km1[i] ].texture() = ints.at(ints.size()/2);

                // Set the initialization flag to true for this vertex
                initialized_vertex[ j_km1[i] ] = true;
            }
        }
    }
}

} // namespace ceres_slam
