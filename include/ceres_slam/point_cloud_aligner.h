#ifndef CERES_SLAM_POINT_CLOUD_ALIGNER_H_
#define CERES_SLAM_POINT_CLOUD_ALIGNER_H_

#include <vector>

#include <ceres_slam/geometry.h>
#include <ceres_slam/stereo_camera.h>

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
    //! Vector type
    typedef Vector3D<double> Vector;
    //! Camera type
    typedef StereoCamera<double> Camera;

    //! Default constructor
    PointCloudAligner() { }

    //! Compute the transformation that aligns
    //! the origin cloud (pts_0) onto the destination cloud (pts_1).
    SE3 compute_transformation(const std::vector<Point>& pts_0,
                               const std::vector<Point>& pts_1);

    //! Compute the transformation that aligns
    //! the origin cloud (pts_0) onto the destination cloud (pts_1)
    //! and remove outliers from the two clouds.
    SE3 compute_transformation_and_inliers(std::vector<Point>& pts_0,
                                           std::vector<Point>& pts_1,
                                           Camera::ConstPtr camera,
                                           int num_iters = 400,
                                           double thresh = 25);
};

} // namespace ceres_slam

#endif // CERES_SLAM_POINT_CLOUD_ALIGNER_H_
