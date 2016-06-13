#ifndef CERES_SLAM_IMAGE_ERROR_H_
#define CERES_SLAM_IMAGE_ERROR_H_

#include <ceres/ceres.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres_slam/geometry/geometry.h>
#include <ceres_slam/stereo_camera.h>

namespace ceres_slam {

//! Per-pixel inverse-compositional photometric error
class ImageError : public ceres::SizedCostFunction<
                       1,   // Residual dimension
                       12,  // Compact SE(3) vehicle pose (3 trans + 9 rot)
                       1>   // Disparity measurement dimension
{
   public:
    typedef SE3Group<double> SE3;
    typedef Point3D<double> Point;
    typedef StereoCamera<double> Camera;
    typedef typename Camera::Observation Observation;
    typedef typename Camera::ProjectionJacobian ProjectionJacobian;
    typedef typename Camera::TriangulationJacobian TriangulationJacobian;

    ImageError(const Camera::ConstPtr camera, const cv::Ptr<cv::Mat> ref_img,
               const cv::Ptr<cv::Mat> track_img,
               const cv::Ptr<cv::Mat> ref_gradx,
               const cv::Ptr<cv::Mat> ref_grady, const double u_ref,
               const double v_ref)
        : camera_(camera),
          ref_img_(ref_img),
          track_img_(track_img),
          ref_gradx_(ref_gradx),
          ref_grady_(ref_grady),
          u_ref_(u_ref),
          v_ref_(v_ref) {}

    virtual ~ImageError() {}

    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
        // Camera pose relative to reference frame
        Eigen::Map<const SE3> T_track_ref(&parameters[0][0]);

        // Reference pixel (u, v, d, I)
        const double d_ref = parameters[1][0];
        Observation uvd_ref(u_ref_, v_ref_, d_ref);

        // Project into the tracking image
        TriangulationJacobian tri_jac;
        Point xyz_ref = camera_->triangulate(uvd_ref, &tri_jac);
        Point xyz_track = T_track_ref * xyz_ref;

        ProjectionJacobian proj_jac;
        Observation uvd_track = camera_->project(xyz_track, &proj_jac);
        double u_track = uvd_track(0);
        double v_track = uvd_track(1);

        // Image bound check
        if (!check_bounds(track_img_, u_track, v_track)) {
            return false;
        }

        // Compute the intensity residual
        double int_ref = interpolate(ref_img_, u_ref_, v_ref_);
        double int_track = interpolate(track_img_, u_track, v_track);
        residuals[0] = int_track - int_ref;

        // Compute the Jacobians if requested
        if (jacobians != NULL) {
            // Jacobian w.r.t. T_track_ref
            if (jacobians[0] != NULL) {
                
            }

            // Jacobian w.r.t. d_ref
            if (jacobians[1] != NULL) {
            }
        }

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const Camera::ConstPtr camera,
                                       const cv::Ptr<cv::Mat> ref_img,
                                       const cv::Ptr<cv::Mat> track_img,
                                       const cv::Ptr<cv::Mat> ref_gradx,
                                       const cv::Ptr<cv::Mat> ref_grady,
                                       const double u_ref, const double v_ref) {
        return (new ImageError(camera, ref_img, track_img, ref_gradx, ref_grady,
                               u_ref, v_ref));
    }

   private:
    Camera::ConstPtr camera_;
    cv::Ptr<cv::Mat> ref_img_;
    cv::Ptr<cv::Mat> track_img_;
    cv::Ptr<cv::Mat> ref_gradx_;
    cv::Ptr<cv::Mat> ref_grady_;
    double u_ref_;
    double v_ref_;

    //! Bilinearly interpolate at real-valued pixel indices
    double interpolate(const cv::Ptr<cv::Mat> img, const double u,
                       const double v) const {
        // Nearest neighbour for now
        int u_nn = round(u);
        int v_nn = round(v);
        return img->at<double>(u_nn, v_nn);
    }

    //! Check if pixel coordinates are outside image bounds
    bool check_bounds(const cv::Ptr<cv::Mat> img, const double u,
                      const double v) const {
        return (u >= 0. && u < (double)img->rows && v >= 0. &&
                v < (double)img->cols);
    }
};
}

#endif /* end of include guard: CERES_SLAM_IMAGE_ERROR_H_ */
