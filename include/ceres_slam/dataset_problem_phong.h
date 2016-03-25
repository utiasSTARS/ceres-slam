#ifndef CERES_SLAM_DATASET_PROBLEM_PHONG_H_
#define CERES_SLAM_DATASET_PROBLEM_PHONG_H_

#include <vector>
#include <string>
#include <sstream>

#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/lighting.h>
#include <ceres_slam/geometry.h>

namespace ceres_slam {

//! Class for reading simulated datasets from file
class DatasetProblemPhong {
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
    //! Vertex type
    typedef Vertex3D<double> Vertex;

    //! Default constructor
    DatasetProblemPhong(bool dir_light = false)
        : directional_light(dir_light) {}

    //! Camera model
    Camera::Ptr camera;

    //! Timestamps (measured)
    std::vector<double> t;
    //! Number of states to optimize
    uint num_states;
    //! Number of map points to optimize
    uint num_vertices;
    //! Number of materials to optimize
    uint num_materials;

    //! Camera poses in base frame (to be estimated)
    std::vector<SE3> poses;

    //! Map vertices in base frame (to be estimated)
    std::vector<Vertex> map_vertices;
    //! Map vertex IDs in stereo_obs_list
    std::vector<uint> vertex_ids;
    //! True if map vertex j has been initialized
    std::vector<bool> initialized_vertex;
    //! Map vertex material IDs in stereo_obs_list
    std::vector<uint> material_ids;

    //! Use directional light?
    bool directional_light;
    //! Light source position in base frame (to be estimated)
    Point light_pos;
    //! Light source position in base frame (to be estimated)
    Vector light_dir;

    //! Materials (to be estimated)
    std::vector<Material<double>::Ptr> materials;
    //! Textures (to be estimated)
    std::vector<Texture<double>::Ptr> textures;

    //! List of stereo observations
    std::vector<Camera::Observation> stereo_obs_list;
    //! Variance of stereo observations
    Camera::ObservationVariance stereo_obs_var;
    //! List of observation intensities
    std::vector<double> int_list;
    //! Variance of observation intensities
    double int_var;
    //! List of normal observations
    std::vector<Vector> normal_obs_list;
    //! Variance of normal observations
    Vector::Variance normal_obs_var;

    //! Read dataset from a CSV file
    /*!
        Assuming first row is num_states, num_vertices,
        second row is intrinsics,
        and remaining rows are observations of the form [t,j,u,v,d,I]
        where t: timestamp
              j: point index
              (u,v): left image coordinates
              d: disparity
              I: intensity
    */
    const bool read_csv(const std::string filename);

    //! Write result to a CSV file
    const bool write_csv(const std::string filename) const;

    //! Return list of indices corresponding to a specified state index
    const std::vector<uint> obs_indices_at_state(int k) const;

    //! Return list of indices corresponding to a specified feature index
    const std::vector<uint> obs_indices_for_feature(int j) const;

    //! Return list of indices corresponding to a specified material ID
    const std::vector<uint> obs_indices_for_material(int m) const;

    //! Generate initial guess for poses and map points
    //! using scalar-weighted point cloud alignment for stereo VO
    void compute_initial_guess(uint k1 = 0, uint k2 = 0);

   private:
    //! List of lists of indices corresponding to each state index
    std::vector<std::vector<uint>> state_indices_;

    //! List of lists of indices corresponding to each feature index
    std::vector<std::vector<uint>> feature_indices_;

    //! List of lists of indices corresponding to each material index
    std::vector<std::vector<uint>> material_indices_;

};  // class DatasetProblemPhong

}  // namespace ceres_slam

#endif  // CERES_SLAM_DATASET_PROBLEM_PHONG_H_
