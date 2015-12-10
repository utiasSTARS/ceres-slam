#ifndef CERES_SLAM_POINT_CLOUD_ALIGNER_H_
#define CERES_SLAM_POINT_CLOUD_ALIGNER_H_

#include <vector>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

//! Align two point clouds with known data association
class PointCloudAligner {
public:
    //! SO(3) type
    typedef SO3Group<double> SO3;
    //! SE(3) type
    typedef SE3Group<double> SE3;
    //! Point type
    typedef Point3D<double> Point;

    //! Default constructor
    PointCloudAligner();

    //! Set the input point clouds
    void set_input_clouds(std::vector<Point>* const pts_0,
                          std::vector<Point>* const pts_1);

    //! Compute the transformation that aligns
    //! the origin cloud onto the destimation cloud.
    SE3 compute_transformation();

private:
    std::vector<Point>* pts_0_; //!< Origin point cloud
    std::vector<Point>* pts_1_; //!< Desination point cloud
};

} // namespace ceres_slam

#endif // CERES_SLAM_POINT_CLOUD_ALIGNER_H_
