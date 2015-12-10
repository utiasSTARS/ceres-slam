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
    PointCloudAligner() { }

    //! Compute the transformation that aligns
    //! the origin cloud (pts_0) onto the destination cloud (pts_1).
    SE3 compute_transformation(std::vector<Point>* const pts_0,
                               std::vector<Point>* const pts_1);
};

} // namespace ceres_slam

#endif // CERES_SLAM_POINT_CLOUD_ALIGNER_H_
