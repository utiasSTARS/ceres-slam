#ifndef CERES_SLAM_DATASET_PROBLEM_H_
#define CERES_SLAM_DATASET_PROBLEM_H_

#include <vector>
#include <string>
#include <sstream>

#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/lighting.h>
#include <ceres_slam/geometry.h>

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
    DatasetProblem(bool reinitialize_points = true)
        : reinitialize_points_(reinitialize_points) {}

    //! Camera model
    Camera::Ptr camera;

    //! Pose ID
    std::vector<unsigned int> k;
    //! Number of states to optimize
    unsigned int num_states;
    //! Number of map points to optimize
    unsigned int num_points;

    //! Camera poses in base frame (to be estimated)
    std::vector<SE3> poses;

    //! Map points in base frame (to be estimated)
    std::vector<Point> map_points;
    //! Map point IDs in stereo_obs_list
    std::vector<unsigned int> point_ids;
    //! True if map point j has been initialized
    std::vector<bool> initialized_point;
    //! Map point material IDs in stereo_obs_list
    std::vector<unsigned int> material_ids;

    //! List of stereo observations
    std::vector<Camera::Observation> stereo_obs_list;
    //! Variance of stereo observations
    Camera::ObservationVariance stereo_obs_var;

    //! Read dataset from a CSV file
    /*!
        Assuming first row is num_states, num_points,
        second row is intrinsics,
        and remaining rows are observations of the form [k,j,u,v,d]
        where k: pose index
              j: point index
              (u,v): left image coordinates
              d: disparity
    */
    const bool read_csv(const std::string filename);

    //! Write result to a CSV file
    const bool write_csv(const std::string filename) const;

    //! Return list of indices corresponding to a specified state index
    const std::vector<unsigned int> obs_indices_at_state(unsigned int k) const;

    //! Return list of indices corresponding to a specified feature index
    const std::vector<unsigned int> obs_indices_for_feature(
        unsigned int j) const;

    //! Generate initial guess for poses and map points
    //! using scalar-weighted point cloud alignment for stereo VO
    void compute_initial_guess(unsigned int k1 = 0, unsigned int k2 = 0);

   private:
    //! List of lists of indices corresponding to each state index
    std::vector<std::vector<unsigned int>> state_indices_;
    //! List of lists of indices corresponding to each feature index
    std::vector<std::vector<unsigned int>> feature_indices_;
    //! Should we reinitialize re-observed points when computing the initial
    //! guess?
    bool reinitialize_points_;

};  // class DatasetProblem

}  // namespace ceres_slam

#endif  // CERES_SLAM_DATASET_PROBLEM_H_
