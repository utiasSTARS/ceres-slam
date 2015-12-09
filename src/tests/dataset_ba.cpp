#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres_slam/dataset_problem.h>

int main(int argc, char** argv) {
    std::string filename(argv[1]);

    ceres_slam::DatasetProblem problem;
    problem.read_csv(filename);

    return EXIT_SUCCESS;
}
