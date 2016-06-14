#ifndef CERES_SLAM_DATASET_PROBLEM_SUN_H_
#define CERES_SLAM_DATASET_PROBLEM_SUN_H_

#include <sstream>
#include <string>
#include <vector>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/lighting/lighting.hpp>
#include <ceres_slam/stereo_camera.hpp>

namespace ceres_slam {

//! Class for reading simulated datasets from file
class DatasetProblemSun {
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
    DatasetProblemSun() {}

    //! Camera model
    Camera::Ptr camera;

    //! Pose ID
    std::vector<uint> state_ids;
    //! Number of states to optimize
    uint num_states;
    //! Number of map points to optimize
    uint num_points;

    //! Camera poses in base frame (to be estimated)
    std::vector<SE3> poses;

    //! Map points in base frame (to be estimated)
    std::vector<Point> map_points;
    //! Map point IDs in stereo_obs_list
    std::vector<uint> point_ids;
    //! True if map point j has been initialized
    std::vector<bool> initialized_point;
    //! Map point material IDs in stereo_obs_list
    std::vector<uint> material_ids;

    //! List of stereo observations
    std::vector<Camera::Observation> stereo_obs_list;
    //! Variance of stereo observations
    Camera::ObservationVariance stereo_obs_var;
    //! List of sun direction observations
    std::vector<Vector> sun_obs_list;
    //! Variance of sun direction observations
    Vector::Variance sun_obs_var;
    //! True if state k has a sun observation
    std::vector<bool> state_has_sun_obs;
    //! Sun direction in the global frame
    Vector sun_dir_g;

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
    const std::vector<uint> obs_indices_at_state(uint k) const;

    //! Return list of indices corresponding to a specified feature index
    const std::vector<uint> obs_indices_for_feature(uint j) const;

    //! Reset initialization flags for all points
    void reset_points();

    //! Generate initial guess for poses and map points
    //! using scalar-weighted point cloud alignment for stereo VO
    void compute_initial_guess(uint k1 = 0, uint k2 = 0);

   private:
    //! List of lists of indices corresponding to each state index
    std::vector<std::vector<uint>> state_indices_;
    //! List of lists of indices corresponding to each feature index
    std::vector<std::vector<uint>> feature_indices_;

};  // class DatasetProblemSun

}  // namespace ceres_slam

#endif  // CERES_SLAM_DATASET_PROBLEM_SUN_H_
