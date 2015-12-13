#include <ceres_slam/point_cloud_aligner.h>

#include <vector>

#include <ceres_slam/geometry.h>

#include <Eigen/Eigenvalues> // For SVD

namespace ceres_slam {

PointCloudAligner::SE3 PointCloudAligner::compute_transformation(
    const std::vector<Point>& pts_0,
    const std::vector<Point>& pts_1) {

    if(pts_0.size() != pts_1.size()) {
        std::cerr << "Error in PointCloudAligner::compute_transformation -- "
                  << "pts_0 and pts_1 are different sizes.";
        return PointCloudAligner::SE3();
    }

    // Compute the centroids p_0 and p_1 of each cloud
    Point::Cartesian p_0_cart = Point::Cartesian::Zero();
    for(Point p : pts_0) {
        p_0_cart += p.cartesian();
    }
    p_0_cart /= double(pts_0.size());
    Point p_0(p_0_cart);

    Point::Cartesian p_1_cart = Point::Cartesian::Zero();
    for(Point p : pts_1) {
        p_1_cart += p.cartesian();
    }
    p_1_cart /= double(pts_1.size());
    Point p_1(p_1_cart);

    // std::cout << "p_0: " << p_0 << std::endl;
    // std::cout << "p_1: " << p_1 << std::endl;

    // Compute W_1_0
    Eigen::Matrix3d W_1_0 = Eigen::Matrix3d::Zero();
    for(unsigned int i = 0; i < pts_0.size(); ++i) {
        W_1_0 += (pts_1[i] - p_1).cartesian()
                    * (pts_0[i] - p_0).cartesian().transpose();
    }
    W_1_0 /= double(pts_0.size());

    // Compute rotation
    Eigen::JacobiSVD<SO3::TransformationMatrix>
        svd(W_1_0, Eigen::ComputeThinU | Eigen::ComputeThinV);
    SO3::TransformationMatrix
        middle = SO3::TransformationMatrix::Identity();
    middle(2,2) = svd.matrixV().determinant() * svd.matrixU().determinant();

    SO3 C_1_0(svd.matrixU() * middle * svd.matrixV().transpose());

    // Compute translation
    PointCloudAligner::SE3::Vector r_1_0_1 = p_1 - C_1_0 * p_0;

    // Return final transformation
    return SE3(C_1_0, r_1_0_1);
}

PointCloudAligner::SE3 PointCloudAligner::compute_transformation_and_inliers(
    std::vector<PointCloudAligner::Point>& pts_0,
    std::vector<PointCloudAligner::Point>& pts_1,
    PointCloudAligner::Camera::ConstPtr camera,
    int num_iters, double thresh) {

    // Uniformly distributed integers in [a,b], NOT [a,b)
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<unsigned int>
        idx_selector(0, pts_0.size()-1);
    unsigned int rand_idx[3];

    std::vector<PointCloudAligner::Point> test_pts_0, test_pts_1;
    std::vector<unsigned int> inlier_idx, best_inlier_idx;
    PointCloudAligner::SE3 best_T_1_0;

    // 3-point RANSAC algorithm
    for(int ransac_iter = 0; ransac_iter < num_iters; ++ransac_iter) {
        // std::cout << "RANSAC iter = " << ransac_iter << std::endl;
        // Get 3 random, unique indices
        rand_idx[0] = idx_selector(rng);

        rand_idx[1] = idx_selector(rng);
        while(rand_idx[1] == rand_idx[0])
            rand_idx[1] = idx_selector(rng);

        rand_idx[2] = idx_selector(rng);
        while(rand_idx[2] == rand_idx[0] || rand_idx[2] == rand_idx[1])
            rand_idx[2] = idx_selector(rng);

        // Bundle test points
        test_pts_0.clear();
        test_pts_1.clear();
        for(unsigned int i = 0; i < 3; ++i) {
            test_pts_0.push_back(pts_0[ rand_idx[i] ]);
            test_pts_1.push_back(pts_1[ rand_idx[i] ]);
        }

        // Compute minimal transformation estimate
        // std::cout << "test_pts_0" << std::endl;
        // for(PointCloudAligner::Point p : test_pts_0)
        //     std::cout << p << std::endl;
        // std::cout << "test_pts_1" << std::endl;
        // for(PointCloudAligner::Point p : test_pts_1)
        //     std::cout << p << std::endl;

        PointCloudAligner::SE3 T_1_0 =
            compute_transformation(test_pts_0, test_pts_1);
        //    std::cout << "T_1_0 = " << std::endl << T_1_0 << std::endl;

        // Classify points and get inlier and outlier indices
        double error;
        inlier_idx.clear();
        for(unsigned int i = 0; i < pts_0.size(); ++i) {
            error = (camera->project(pts_1[i])
                     - camera->project(T_1_0 * pts_0[i])).squaredNorm();
            // std::cout << "error = " << error << std::endl;
            if(error < thresh) {
                inlier_idx.push_back(i);
            }
        }

        // Keep track of the best (largest) inlier set and transformation
        if(inlier_idx.size() > best_inlier_idx.size()) {
            best_inlier_idx = inlier_idx;
            best_T_1_0 = T_1_0;
        }
    }

    // Delete outliers
    std::vector<PointCloudAligner::Point> inlier_pts_0, inlier_pts_1;
    for(unsigned int i : best_inlier_idx) {
        inlier_pts_0.push_back(pts_0[i]);
        inlier_pts_1.push_back(pts_1[i]);
    }
    pts_0 = inlier_pts_0;
    pts_1 = inlier_pts_1;

    return best_T_1_0;
}

} // namespace ceres_slam
