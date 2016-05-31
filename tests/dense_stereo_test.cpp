#include <fstream>

#include <ceres/ceres.h>

#include <Eigen/Dense>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ceres_slam/geometry/geometry.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/utils/utils.h>

using SE3 = ceres_slam::SE3Group<double>;
using Camera = ceres_slam::StereoCamera<double>;

int main(int argc, char** argv) {
    SE3 transform_to_estimate;

    double fu = 721.5377;
    double fv = fu;
    double cu = 609.5593;
    double cv = 172.854;
    double b = 0.53715;
    Camera stereo_camera(fu, fv, cu, cv, b);

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
    disp0.convertTo(disp0, CV_32F, 1. / 16.);
    disp0.convertTo(disp0_for_plotting, CV_8U, 255. / 64.);

    cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display", disp0_for_plotting);
    cv::waitKey(0);

    std::cout << "Triangulating good points" << std::endl;
    for (int v = 0; v < disp0.rows; ++v) {
        const float* row_ptr = disp0.ptr<float>(v);
        for (int u = 0; u < disp0.cols; ++u) {
            float d = row_ptr[u];
            if (std::isfinite(d) && d > 0.) {
                Camera::Observation obs(u, v, d);
                obs_list.push_back(obs);
                pts_list.push_back(stereo_camera.triangulate(obs));
            }
        }
    }
    std::cout << "Computed " << pts_list.size() << " valid points out of "
              << disp0.rows * disp0.cols << " total points." << std::endl;

    return 0;
}
