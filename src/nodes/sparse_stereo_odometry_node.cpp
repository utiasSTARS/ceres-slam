#include <iostream>
#include <cstdlib>
#include <memory>
#include <functional>
#include <random>

#include <ceres_slam/geometry.h>
#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/stereo_reprojection_error.h>
#include <ceres_slam/point_cloud_aligner.h>

#include <ceres/ceres.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

//! Frame-to-frame feature-based odometry pipeline,
//! heavily borrowed from image_view/src/nodes/stereo_view.cpp
class SparseStereoOdometryNode {
   public:
    //! Default constructor
    SparseStereoOdometryNode() {
        // Read in parameters
        ros::NodeHandle private_nh("~");
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("approximate_sync", do_approximate_sync_, false);
        private_nh.param("num_features", num_features_, 600);

        // Resolve topic names
        ros::NodeHandle nh;

        std::string camera_ns = nh.resolveName("camera");
        std::string left_ns = ros::names::clean(camera_ns + "/left");
        std::string right_ns = ros::names::clean(camera_ns + "/right");

        std::string left_image_topic =
            ros::names::clean(left_ns + "/image_rect");
        std::string left_info_topic =
            ros::names::clean(left_ns + "/camera_info");
        std::string right_image_topic =
            ros::names::clean(right_ns + "/image_rect");
        std::string right_info_topic =
            ros::names::clean(right_ns + "/camera_info");

        ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s",
                 left_image_topic.c_str(), left_info_topic.c_str(),
                 right_image_topic.c_str(), right_info_topic.c_str());

        // Set up image transport
        image_transport::ImageTransport it(nh);

        // Subscribe to four input topics
        left_image_sub_.subscribe(it, left_image_topic, 1);
        left_info_sub_.subscribe(nh, left_info_topic, 1);
        right_image_sub_.subscribe(it, right_image_topic, 1);
        right_info_sub_.subscribe(nh, right_info_topic, 1);

        // Synchronize input topics. Optionally do approximate sync.
        if (do_approximate_sync_) {
            approximate_sync_.reset(new ApproximateSync(
                ApproximatePolicy(queue_size_), left_image_sub_, left_info_sub_,
                right_image_sub_, right_info_sub_));
            approximate_sync_->registerCallback(
                std::bind(&SparseStereoOdometryNode::imageCallback, this,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4));
        } else {
            exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                                            left_image_sub_, left_info_sub_,
                                            right_image_sub_, right_info_sub_));
            exact_sync_->registerCallback(
                std::bind(&SparseStereoOdometryNode::imageCallback, this,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4));
        }

        // Set up VO image publisher
        std::string vo_image_topic =
            ros::names::clean(camera_ns + "/vo_tracks");
        vo_image_publisher_ = it.advertise(vo_image_topic, 1);
    }

    ~SparseStereoOdometryNode() {}

    //! Stereo image callback
    void imageCallback(const sensor_msgs::ImageConstPtr& left_image,
                       const sensor_msgs::CameraInfoConstPtr& left_info,
                       const sensor_msgs::ImageConstPtr& right_image,
                       const sensor_msgs::CameraInfoConstPtr& right_info) {
        // Initialize the camera model if not already done
        if (camera_ == nullptr) {
            camera_ = ceres_slam::StereoCamera::Ptr(
                new ceres_slam::StereoCamera(left_info, right_info));
        }

        // Convert to CV images
        cv_bridge::CvImagePtr left_cv_ptr;
        cv_bridge::CvImagePtr right_cv_ptr;

        try {
            left_cv_ptr = cv_bridge::toCvCopy(
                left_image, sensor_msgs::image_encodings::MONO8);
            right_cv_ptr = cv_bridge::toCvCopy(
                right_image, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Detect features in stereo pair
        cv::ORB detect(num_features_);

        std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat left_descriptors, right_descriptors;
        detect(left_cv_ptr->image, cv::noArray(), left_keypoints,
               left_descriptors);
        detect(right_cv_ptr->image, cv::noArray(), right_keypoints,
               right_descriptors);

        // Find the closest match for each feature.
        //
        // crossCheck is true, so matcher only returns reciprocal best matches
        // i.e., the left feature's best match in the right image considers
        // the left feature its best match as well.
        //
        // Mutual respect, people. It's the only way.
        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> stereo_matches;

        // NOTE: left is query, right is train in DMatch!!
        matcher.match(left_descriptors, right_descriptors, stereo_matches);

        // Filter for good matches by ensuring they are approximately
        // on the same line and have non-negative disparity.
        // Assumes stereo images have been rectified.
        std::vector<cv::KeyPoint> curr_left_keypoints, curr_right_keypoints;
        cv::Mat curr_left_descriptors, curr_right_descriptors;
        int left_idx, right_idx;

        int y_thresh = 5;  // Max diff in y-coordinate of matched features [px]

        for (uint i = 0; i < stereo_matches.size(); ++i) {
            left_idx = stereo_matches[i].queryIdx;
            right_idx = stereo_matches[i].trainIdx;

            if (abs(left_keypoints[left_idx].pt.y -
                    right_keypoints[right_idx].pt.y) < y_thresh &&
                right_keypoints[right_idx].pt.x <
                    left_keypoints[left_idx].pt.x) {
                curr_left_keypoints.push_back(left_keypoints[left_idx]);
                curr_right_keypoints.push_back(right_keypoints[right_idx]);

                curr_left_descriptors.push_back(left_descriptors.row(left_idx));
                curr_right_descriptors.push_back(
                    right_descriptors.row(right_idx));
            }
        }

        // Compute transform from previous camera to current camera
        if (prev_left_keypoints_.size() == 0) {
            // This is the first frame, so set the initial pose to be identity
            T_curr_map_ = Eigen::Affine3d::Identity();
            publishPose();
        } else {
            // Find matches between previous stereo pair and current stereo pair
            // We'll just reuse the matcher object from before
            // and match between the left images
            std::vector<cv::DMatch> vo_matches;
            matcher.match(prev_left_descriptors_, curr_left_descriptors,
                          vo_matches);

            // Bundle good matched keypoints into StereoCamera::Observation
            // format and triangulate them to 3D points
            int prev_idx, curr_idx;

            std::vector<ceres_slam::StereoCamera::Observation> prev_obs,
                curr_obs;
            std::vector<ceres_slam::StereoCamera::Point> prev_pts, curr_pts;

            ceres_slam::StereoCamera::Observation temp_obs;

            for (uint i = 0; i < vo_matches.size(); ++i) {
                prev_idx = vo_matches[i].queryIdx;
                curr_idx = vo_matches[i].trainIdx;

                temp_obs << prev_left_keypoints_[prev_idx].pt.x,
                    prev_left_keypoints_[prev_idx].pt.y,
                    prev_right_keypoints_[prev_idx].pt.x,
                    prev_right_keypoints_[prev_idx].pt.y;
                prev_obs.push_back(temp_obs);
                prev_pts.push_back(camera_->observationToPoint(temp_obs));

                temp_obs << curr_left_keypoints[curr_idx].pt.x,
                    curr_left_keypoints[curr_idx].pt.y,
                    curr_right_keypoints[curr_idx].pt.x,
                    curr_right_keypoints[curr_idx].pt.y;
                curr_obs.push_back(temp_obs);
                curr_pts.push_back(camera_->observationToPoint(temp_obs));
            }

            // Filter outliers and generate initial guess with RANSAC
            Eigen::Affine3d T_curr_prev;
            std::vector<uint> inlier_idx =
                doRANSAC(prev_pts, curr_pts, T_curr_prev);

            std::vector<ceres_slam::StereoCamera::Observation> prev_inlier_obs,
                curr_inlier_obs;
            std::vector<ceres_slam::StereoCamera::Point> prev_inlier_pts,
                curr_inlier_pts;

            for (uint i = 0; i < inlier_idx.size(); ++i) {
                prev_inlier_obs.push_back(prev_obs[inlier_idx[i]]);
                curr_inlier_obs.push_back(curr_obs[inlier_idx[i]]);

                prev_inlier_pts.push_back(prev_pts[inlier_idx[i]]);
                curr_inlier_pts.push_back(curr_pts[inlier_idx[i]]);
            }

            // Optimize over inliers with Ceres
            ceres::Problem problem;

            ceres::Solver::Options solver_options;
            solver_options.minimizer_progress_to_stdout = true;
            solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

            ceres::Solver::Summary summary;

            ceres_slam::StereoCamera::ObservationVariance variance =
                4 * ceres_slam::StereoCamera::ObservationVariance::Ones();

            // Need to convert to the representation that Ceres uses internally
            double prev_trans[3] = {0, 0, 0};
            double prev_rot[3] = {0, 0, 0};  // axis-angle

            double curr_rot[3];
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> C_curr_prev =
                T_curr_prev.linear();
            ceres::RotationMatrixToAngleAxis<double>(C_curr_prev.data(),
                                                     curr_rot);

            // StereoReprojectionError assumes the translation is untransformed,
            // but Eigen::Affine3d stores it as the transformed version
            // so we need to transform it back first
            Eigen::Vector3d t_curr_prev =
                -C_curr_prev.transpose() * T_curr_prev.translation();
            double curr_trans[3];
            curr_trans[0] = t_curr_prev(0);
            curr_trans[1] = t_curr_prev(1);
            curr_trans[2] = t_curr_prev(2);

            for (uint i = 0; i < inlier_idx.size(); ++i) {
                // Add residual block for the previous frame observation
                ceres::CostFunction* prev_stereo_cost =
                    ceres_slam::StereoReprojectionError::Create(
                        camera_, prev_inlier_obs[i], variance);

                problem.AddResidualBlock(prev_stereo_cost, NULL, prev_trans,
                                         prev_rot, prev_inlier_pts[i].data());

                // Add residual block for the current frame observation
                ceres::CostFunction* curr_stereo_cost =
                    ceres_slam::StereoReprojectionError::Create(
                        camera_, curr_inlier_obs[i], variance);
                problem.AddResidualBlock(curr_stereo_cost, NULL, curr_trans,
                                         curr_rot, curr_inlier_pts[i].data());
            }

            Solve(solver_options, &problem, &summary);

            // Now put the solution back in the format we want
            ceres::AngleAxisToRotationMatrix<double>(curr_rot,
                                                     C_curr_prev.data());
            T_curr_prev.linear() = C_curr_prev;

            t_curr_prev(0) = curr_trans[0];
            t_curr_prev(1) = curr_trans[1];
            t_curr_prev(2) = curr_trans[2];
            T_curr_prev.translation() = -C_curr_prev * t_curr_prev;

            // Update and publish
            T_curr_map_ = T_curr_prev * T_curr_map_;
            publishPose();
            publishVOTracks(prev_inlier_obs, curr_inlier_obs);
        }

        prev_left_keypoints_ = curr_left_keypoints;
        prev_right_keypoints_ = curr_right_keypoints;
        prev_left_descriptors_ = curr_left_descriptors;
        prev_right_descriptors_ = curr_right_descriptors;
        prev_left_cv_ptr_ = left_cv_ptr;
        prev_right_cv_ptr_ = right_cv_ptr;
    }

    //! VO track publisher
    void publishVOTracks(
        cv_bridge::CvImagePtr image_ptr,
        std::vector<ceres_slam::StereoCamera::Observation>& prev_obs,
        std::vector<ceres_slam::StereoCamera::Observation>& curr_obs) {
        int thickness = 2;
        int line_type = 8;
        cv::Scalar color(0, 255, 255);

        for (int i = 0; i < prev_obs.size(); ++i) {
            cv::Point start(prev_obs[i](0), prev_obs[i](1));
            cv::Point end(curr_obs[i](0), curr_obs[i](1));
            cv::line(image_ptr->image, start, end, color, thickness, line_type);
        }

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", image_ptr->image)
                .toImageMsg();
    }

    //! Pose publisher
    void publishPose() {
        // Convert to TF pose
        tf::Pose tf_pose;
        tf::poseEigenToTF(T_curr_map_, tf_pose);

        // Publish
        pose_broadcaster_.sendTransform(
            tf::StampedTransform(tf_pose, ros::Time::now(), "map", "camera"));
    }

   private:
    //! Message queue size
    int queue_size_;
    //! Approximate or exact sync
    bool do_approximate_sync_;
    //! Number of features to detect and track
    int num_features_;

    //! Camera model
    ceres_slam::StereoCamera::Ptr camera_;

    //! Camera pose
    ceres_slam::SE3 T_curr_map_;

    //! Storage for previous frame's keypoints
    std::vector<cv::KeyPoint> prev_left_keypoints_, prev_right_keypoints_;
    //! Storage for previous frame's descriptors
    cv::Mat prev_left_descriptors_, prev_right_descriptors_;
    //! Storage for previous frame's images
    cv_bridge::CvImagePtr prev_left_cv_ptr_, prev_right_cv_ptr_;

    //! Image subscribers
    image_transport::SubscriberFilter left_image_sub_, right_image_sub_;
    //! Camera info subscribers
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_,
        right_info_sub_;

    //! Exact sync policy
    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
        sensor_msgs::CameraInfo> ExactPolicy;
    //! Approximat sync policy
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
        sensor_msgs::CameraInfo> ApproximatePolicy;

    //! Exact sync message filter
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    //! Approximate sync message filter
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

    //! Exact synchronizer
    std::shared_ptr<ExactSync> exact_sync_;
    //! Approximate synchronizer
    std::shared_ptr<ApproximateSync> approximate_sync_;

    //! Pose broadcaster
    tf::TransformBroadcaster pose_broadcaster_;
    //! VO image publisher
    image_transport::Publisher vo_image_publisher_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_node", ros::init_options::AnonymousName);

    if (ros::names::remap("camera") == "camera") {
        ROS_WARN(
            "'camera' has not been remapped! Example command-line usage:\n"
            "\t$ rosrun ceres_slam odometry_node camera:=xb3");
    }

    SparseStereoOdometryNode odometry_node;

    ros::spin();

    return EXIT_SUCCESS;
}
