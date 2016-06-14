#include <ceres/ceres.h>

#include <Eigen/Dense>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/image_error.hpp>
#include <ceres_slam/perturbations.hpp>
#include <ceres_slam/stereo_camera.hpp>
#include <ceres_slam/utils/utils.hpp>

using SE3 = ceres_slam::SE3Group<double>;
using Camera = ceres_slam::StereoCamera<double>;

int main(int argc, char** argv) {
    SE3 transform_to_estimate;

    double fu = 721.5377;
    double fv = fu;
    double cu = 609.5593;
    double cv = 172.854;
    double b = 0.53715;
    Camera::Ptr stereo_camera = std::make_shared<Camera>(fu, fv, cu, cv, b);

    std::vector<Camera::Observation> obs_list;
    std::vector<Camera::Point> pts_list;

    std::string impath =
        "/home/leeclement/osx_desktop/datasets/2011_09_26/"
        "2011_09_26_drive_0019_sync/";

    cv::Mat left0, right0, left1, right1;
    left0 = cv::imread(impath + "image_00/data/0000000000.png",
                       CV_LOAD_IMAGE_GRAYSCALE);
    right0 = cv::imread(impath + "image_01/data/0000000000.png",
                        CV_LOAD_IMAGE_GRAYSCALE);
    left1 = cv::imread(impath + "image_00/data/0000000001.png",
                       CV_LOAD_IMAGE_GRAYSCALE);
    right1 = cv::imread(impath + "image_01/data/0000000001.png",
                        CV_LOAD_IMAGE_GRAYSCALE);

    if (!left0.data || !right0.data || !left1.data || !right1.data) {
        std::cerr << "Nooooooo\n" << impath << std::endl;
        return -1;
    }

    cv::StereoSGBM sgbm(0, 64, 15);

    cv::Mat disp0, disp0_for_plotting;
    sgbm(left0, right0, disp0);
    disp0.convertTo(disp0, CV_64F, 1. / 16.);

    cv::Mat left0_gradx, left0_grady;
    left0.convertTo(left0, CV_64F, 1. / 255.);
    left1.convertTo(left1, CV_64F, 1. / 255.);
    // TODO: CHANGE THIS BACK TO left0
    cv::Scharr(left1, left0_gradx, -1, 1, 0);
    cv::Scharr(left1, left0_grady, -1, 0, 1);

    // std::cout << left0.at<double>(0,0) << std::endl;

    cv::Ptr<cv::Mat> left0_ptr(&left0);
    cv::Ptr<cv::Mat> left1_ptr(&left1);
    cv::Ptr<cv::Mat> left0_gradx_ptr(&left0_gradx);
    cv::Ptr<cv::Mat> left0_grady_ptr(&left0_grady);

    // disp0.convertTo(disp0_for_plotting, CV_8U, 255. / 64.);
    // cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display", disp0_for_plotting);
    // cv::waitKey(0);

    std::cout << "Initial estimate: " << transform_to_estimate << "\n";

    std::cout << "Creating problem...\n";

    ceres::Problem problem;
    ceres::LocalParameterization* se3_perturbation =
        ceres_slam::SE3Perturbation::Create();

    std::cout << "Adding residual blocks...\n";
    for (int v = 0; v < disp0.rows; ++v) {
        double* row_ptr = disp0.ptr<double>(v);
        for (int u = 0; u < disp0.cols; ++u) {
            double d = row_ptr[u];
            if (std::isfinite(d) && d > 0.) {
                ceres::CostFunction* image_cost =
                    ceres_slam::ImageError::Create(
                        stereo_camera, left0_ptr, left1_ptr, left0_gradx_ptr,
                        left0_grady_ptr, (double)u, (double)v);
                problem.AddResidualBlock(image_cost, NULL,
                                         transform_to_estimate.data(),
                                         row_ptr + u);
            }
        }
    }

    problem.SetParameterization(transform_to_estimate.data(), se3_perturbation);

    std::cout << "Solving...\n";
    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    // solver_options.num_threads = 8;
    // solver_options.num_linear_solver_threads = 8;
    // solver_options.max_num_iterations = 1000;
    solver_options.use_nonmonotonic_steps = true;
    // solver_options.trust_region_strategy_type = ceres::DOGLEG;
    // solver_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    // solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // solver_options.check_gradients = true;

    ceres::Solver::Summary summary;

    Solve(solver_options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl << std::endl;

    std::cout << "Final estimate: " << transform_to_estimate << "\n";

    return 0;
}
