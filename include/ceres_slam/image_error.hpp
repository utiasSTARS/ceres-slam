#ifndef CERES_SLAM_IMAGE_ERROR_H_
#define CERES_SLAM_IMAGE_ERROR_H_

#include <ceres/ceres.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/stereo_camera.hpp>

namespace ceres_slam {

//! Per-pixel inverse-compositional photometric error
class ImageError : public ceres::SizedCostFunction<
                       1,   // Residual dimension
                       12,  // Compact SE(3) vehicle pose (3 trans + 9 rot)
                       1>   // Disparity measurement dimension
{
   public:
    typedef SE3Group<double> SE3;
    typedef typename SE3::TransformJacobian TransformJacobian;
    typedef Point3D<double> Point;
    typedef StereoCamera<double> Camera;
    typedef typename Camera::Observation Observation;
    typedef typename Camera::ProjectionJacobian ProjectionJacobian;
    typedef typename Camera::TriangulationJacobian TriangulationJacobian;

    ImageError(const Camera::ConstPtr camera, const cv::Ptr<cv::Mat> ref_img,
               const cv::Ptr<cv::Mat> track_img,
               const cv::Ptr<cv::Mat> track_gradx,
               const cv::Ptr<cv::Mat> track_grady, const double u_ref,
               const double v_ref)
        : camera_(camera),
          ref_img_(ref_img),
          track_img_(track_img),
          track_gradx_(track_gradx),
          track_grady_(track_grady),
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

        TransformJacobian trans_jac;
        Point xyz_track = T_track_ref.transform(xyz_ref, &trans_jac);

        ProjectionJacobian proj_jac;
        Observation uvd_track = camera_->project(xyz_track, &proj_jac);
        double u_track = uvd_track(0);
        double v_track = uvd_track(1);

        // Image bound check
        if (check_bounds(track_img_, u_track, v_track)) {
            // Compute the intensity residual
            double int_ref = interpolate(ref_img_, u_ref_, v_ref_);
            double int_track = interpolate(track_img_, u_track, v_track);
            residuals[0] = int_track - int_ref;

            // std::cout << "ref: ( " << u_ref_ << ", " << v_ref_ << ", " <<
            // int_ref
            // << " ) | " << "track: ( " << u_track << ", " << v_track << ", "
            // <<
            // int_track << " ) \n\n";

            // Compute the Jacobians if requested
            if (jacobians != NULL) {
                // Jacobian w.r.t. image coordinates (1 x 2)
                Eigen::Matrix<double, 1, 2> de_by_du;
                de_by_du << interpolate(track_gradx_, u_track, v_track),
                    interpolate(track_grady_, u_track, v_track);

                // Jacobian w.r.t. T_track_ref (1 x 12)
                if (jacobians[0] != NULL) {
                    Eigen::Map<Eigen::Matrix<double, 1, 12>> de_by_dT(
                        &jacobians[0][0]);
                    de_by_dT =
                        de_by_du * proj_jac.block(0, 0, 2, 3) * trans_jac;

                    // std::cout << "\n\n" << de_by_du << "\n\n" <<
                    // proj_jac.block(0, 0, 2, 3) << "\n\n" << trans_jac <<
                    // "\n\n"
                    // << de_by_dT << "\n\n";
                }

                // Jacobian w.r.t. d_ref (1 x 1)
                // Need to access element 0 because the product returns an Eigen
                // matrix, which can't be cast directly to double.
                if (jacobians[1] != NULL) {
                    jacobians[1][0] = (de_by_du * proj_jac.block(0, 0, 2, 3) *
                                       T_track_ref.rotation().matrix() *
                                       trans_jac.block(2, 0, 3, 1))(0);
                }
            }
        } else {
            // Ideally, the cost function should just return false in these
            // cases since the residuals is undefined for out-of-bounds pixels.
            // However, if the cost function returns false during the initial
            // evaluation, the entire optimization fails because ceres won't
            // handle infeasible starts.

            // std::cout << "Residual evaluation failed due to out-of-bounds "
            //              "pixel coordinate: (u, v) = ("
            //           << u_track << ", " << v_track << ")\n";

            residuals[0] = 0;

            if (jacobians != NULL) {
                if (jacobians[0] != NULL) {
                    Eigen::Map<Eigen::Matrix<double, 1, 12>> de_by_dT(
                        &jacobians[0][0]);
                    de_by_dT = Eigen::Matrix<double, 1, 12>::Zero();
                }
                if (jacobians[1] != NULL) {
                    jacobians[1][0] = 0;
                }
            }
        }

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(const Camera::ConstPtr camera,
                                       const cv::Ptr<cv::Mat> ref_img,
                                       const cv::Ptr<cv::Mat> track_img,
                                       const cv::Ptr<cv::Mat> track_gradx,
                                       const cv::Ptr<cv::Mat> track_grady,
                                       const double u_ref, const double v_ref) {
        return (new ImageError(camera, ref_img, track_img, track_gradx,
                               track_grady, u_ref, v_ref));
    }

   private:
    Camera::ConstPtr camera_;
    cv::Ptr<cv::Mat> ref_img_;
    cv::Ptr<cv::Mat> track_img_;
    cv::Ptr<cv::Mat> track_gradx_;
    cv::Ptr<cv::Mat> track_grady_;
    double u_ref_;
    double v_ref_;

    //! Bilinearly interpolate at real-valued pixel indices
    double interpolate(const cv::Ptr<cv::Mat> img, const double u,
                       const double v) const {
        // cv::Mat access is (row, col), i.e. (v, u)

        // Nearest neighbour for now
        int u_nn = round(u);
        int v_nn = round(v);
        return img->at<double>(v_nn, u_nn);  // i, j
    }

    //! Check if pixel coordinates are outside image bounds
    bool check_bounds(const cv::Ptr<cv::Mat> img, const double u,
                      const double v) const {
        // cv::Mat access is (row, col), i.e. (v, u)
        return (u >= 0. && u < (double)img->cols && v >= 0. &&
                v < (double)img->rows);
    }
};
}

#endif /* end of include guard: CERES_SLAM_IMAGE_ERROR_H_ */
