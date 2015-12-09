#include <ceres_slam/dataset_problem.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <string>
#include <sstream>

#include <ceres_slam/geometry.h>
#include <ceres_slam/stereo_camera.h>

bool ceres_slam::DatasetProblem::read_csv(std::string filename) {
    std::string line;
    std::vector<std::string> tokens;
    std::ifstream file(filename);

    // Quit if you can't open the file
    if(!file.is_open()) {
        return false;
    }

    // Read camera intrinsics
    std::getline(file, line);
    tokens = split(line, ',');
    double fu = std::stod(tokens.at(0));
    double fv = std::stod(tokens.at(1));
    double cu = std::stod(tokens.at(2));
    double cv = std::stod(tokens.at(3));
    double b  = std::stod(tokens.at(4));
    camera = std::make_shared<Camera>(fu, fv, cu, cv, b);

    // Read in the observations
    while(std::getline(file, line)) {
        tokens = split(line, ',');
        t.push_back( std::stod(tokens.at(0)) );
        j.push_back( std::stod(tokens.at(1)) );
        double u = std::stod(tokens.at(2));
        double v = std::stod(tokens.at(3));
        double d = std::stod(tokens.at(4));
        obs_list.push_back( Camera::Observation(u,v,d) );
    }

    std::cout << t[t.size()-1] << std::endl << j[j.size()-1] << std::endl
              << obs_list[obs_list.size()-1] << std::endl;

    file.close();
    return true;
}

bool ceres_slam::DatasetProblem::write_csv(std::string filename) {
    return true;
}

std::vector<std::string>
ceres_slam::DatasetProblem::split(std::string str, char delimiter) {
    std::vector<std::string> internal;
    std::stringstream ss(str); // Turn the string into a stream.
    std::string tok;

    while(getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }

    return internal;
}
