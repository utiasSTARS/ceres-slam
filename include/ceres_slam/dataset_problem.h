#ifndef CERES_SLAM_DATASET_PROBLEM_H_
#define CERES_SLAM_DATASET_PROBLEM_H_

#include <vector>
#include <string>
#include <sstream>

#include <ceres_slam/geometry.h>
#include <ceres_slam/stereo_camera.h>

namespace ceres_slam {

//! Class for reading simulated datasets from file
class DatasetProblem {
public:
    //! Camera type
    typedef StereoCamera<double> Camera;
    //! SO(3) type
    typedef SO3Group<double> SO3;
    //! SE(3) type
    typedef SE3Group<double> SE3;
    //! Point type
    typedef Point3D<double> Point;
    //! Vector type
    typedef Vector3D<double> Vector;

    //! Default constructor
    DatasetProblem() { }

    //! Camera model
    Camera::Ptr camera;
    //! Timestamps (measured)
    std::vector<double> t;
    //! Map point IDs in obs_list
    std::vector<int> j;
    //! List of stereo observations
    std::vector<Camera::Observation> obs_list;
    //! Variance of stereo observations
    Camera::ObservationVariance obs_var;
    //! Camera poses in base frame (to be estimated)
    std::vector<SE3> poses;
    //! Map points in base frame (to be estimated)
    std::vector<Point> map_points;

    //! Read dataset from a CSV file
    /*!
        Assuming first row is intrinsics, and remaining rows are observations
        of the form [t,j,u,v,d,I]
        where t: timestemp
              j: point index
              (u,v): left image coordinates
              d: disparity
              I: intensity
    */
    bool read_csv(const std::string filename);

    //! Write result to a CSV file
    bool write_csv(const std::string filename);

private:
    //! Split a delimited string into a vector of tokens
    std::vector<std::string> split(std::string str, char delimiter);

}; // class DatasetProblem

} // namespace ceres_slam

#endif // CERES_SLAM_DATASET_PROBLEM_H_
