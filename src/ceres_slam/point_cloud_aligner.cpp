#include <ceres_slam/point_cloud_aligner.h>

#include <vector>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

PointCloudAligner::PointCloudAligner() {
    pts_0_ = nullptr;
    pts_1_ = nullptr;
}

void PointCloudAligner::set_input_clouds(std::vector<Point>* const pts_0,
                                         std::vector<Point>* const pts_1) {
    pts_0_ = pts_0;
    pts_1_ = pts_1;
}

PointCloudAligner::SE3 PointCloudAligner::compute_transformation() {
    if(pts_0_ != nullptr && pts_1_ != nullptr) {
        unsigned int n_pts = pts_0_->size();

        // Compute the centroids of each cloud
        Point::Cartesian p_0_cart = Point::Cartesian::Zero();
        Point::Cartesian p_1_cart = Point::Cartesian::Zero();
        for(unsigned int i = 0; i < n_pts; ++i) {
            p_0_cart += (*pts_0_)[i].cartesian(); // Ugly syntax :(
            p_1_cart += (*pts_1_)[i].cartesian();
        }
        p_0_cart /= n_pts;
        p_1_cart /= n_pts;

        Point p_0(p_0_cart);
        Point p_1(p_1_cart);

        // Compute W_1_0
        Eigen::Matrix3d W_1_0 = Eigen::Matrix3d::Zero();
        for(unsigned int i = 0; i < n_pts; ++i) {
            W_1_0 += ((*pts_1_)[i] - p_1).cartesian()
                        * ((*pts_0_)[i] - p_0).cartesian().transpose();
        }
        W_1_0 /= n_pts;

        // Compute rotation
        Eigen::JacobiSVD<SO3::TransformationMatrix> svd(W_1_0, Eigen::ComputeThinU
                                                             | Eigen::ComputeThinV);
        SO3::TransformationMatrix middle = SO3::TransformationMatrix::Identity();
        middle(2,2) = svd.matrixU().determinant() * svd.matrixV().determinant();

        SO3 C_1_0(svd.matrixU() * middle * svd.matrixV().transpose());

        // Compute translation
        PointCloudAligner::SE3::Vector r_1_0_1 = p_1 - C_1_0 * p_0;

        // Return final transformation
        return SE3(C_1_0, r_1_0_1);
    }
    else {
        std::cerr << "Input point clouds not set! Returning identity!"
                  << std::endl;
        return SE3();
    }
}

} // namespace ceres_slam
