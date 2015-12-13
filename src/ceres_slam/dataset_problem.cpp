#include <ceres_slam/dataset_problem.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <string>
#include <sstream>

#include <ceres_slam/geometry.h>
#include <ceres_slam/stereo_camera.h>

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
    num_points = std::stoi(tokens.at(1));
    pose_vectors.resize(num_states);
    map_points.resize(num_points);
    for(unsigned int i = 0; i < num_points; ++i) {
        initialized_point.push_back(false);
    }
    std::cerr << "expecting " << num_states << " states"
              << " and " << num_points << " points" << std::endl;

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
    while(std::getline(file, line)) {
        tokens = split(line, ',');
        t.push_back( std::stod(tokens.at(0)) );
        point_ids.push_back( std::stoi(tokens.at(1)) );
        double u = std::stod(tokens.at(2));
        double v = std::stod(tokens.at(3));
        double d = std::stod(tokens.at(4));
        obs_list.push_back( Camera::Observation(u,v,d) );
    }
    std::cerr << "read " << obs_list.size() << " observations" << std::endl;

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

    file.close();
    return true;
}

const bool DatasetProblem::write_csv(std::string filename) const {
    std::ofstream file(filename);

    // Quit if you can't open the file
    if(!file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename << std::endl;
        return false;
    }

    // Convert poses to CSV entries
    for(SE3::TangentVector xi : pose_vectors) {
        file << SE3::exp(xi).str() << std::endl;
    }

    // Convert initialized map points to CSV entries
    for(unsigned int j = 0; j < map_points.size(); ++j) {
        if(initialized_point[j]) {
            file << j << "," << map_points[j].str() << std::endl;
        }
    }

    file.close();
    return true;
}

const std::vector<unsigned int> DatasetProblem::obs_indices_at_state(int k) const {
    return state_indices_.at(k);
}

std::vector<std::string> DatasetProblem::split(std::string str, char del) {
    std::stringstream ss(str); // Copy the string into a stream
    std::vector<std::string> tokens;
    std::string tok;

    while(getline(ss, tok, del)) {
        tokens.push_back(tok);
    }

    return tokens;
}

} // namespace ceres_slam