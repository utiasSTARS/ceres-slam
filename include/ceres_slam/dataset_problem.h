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
    //! Number of states to optimize
    unsigned int num_states;
    //! Number of map points to optimize
    unsigned int num_points;
    //! Camera poses in base frame (to be estimated)
    std::vector<SE3::TangentVector> pose_vectors;
    //! First pose, either identity or given by ground truth
    SE3 first_pose;
    //! Map points in base frame (to be estimated)
    std::vector<Point> map_points;
    //! Map point IDs in obs_list
    std::vector<unsigned int> point_ids;
    //! True if map point j has been initialized
    std::vector<bool> initialized_point;
    //! List of stereo observations
    std::vector<Camera::Observation> obs_list;
    //! Variance of stereo observations
    Camera::ObservationVariance obs_var;

    //! Read dataset from a CSV file
    /*!
        Assuming first row is num_states, num_points,
        second row is intrinsics,
        and remaining rows are observations of the form [t,j,u,v,d,I]
        where t: timestemp
              j: point index
              (u,v): left image coordinates
              d: disparity
              I: intensity
    */
    const bool read_csv(const std::string filename);

    //! Write result to a CSV file
    const bool write_csv(const std::string filename) const;

    //! Return list of indices corresponding to a specified state index
    const std::vector<unsigned int> obs_indices_at_state(int k) const;

    //! Generate initial guess for poses and map points
    //! using scalar-weighted point cloud alignment for stereo VO
    void compute_initial_guess();

private:
    //! Split a delimited string into a vector of tokens
    std::vector<std::string> split(std::string str, char del) const;

    //! List of lists of indices corresponding to each state index
    std::vector<std::vector<unsigned int>> state_indices_;

}; // class DatasetProblem

} // namespace ceres_slam

#endif // CERES_SLAM_DATASET_PROBLEM_H_
