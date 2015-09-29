#include <iostream>
#include <cstdlib>
#include <memory>
#include <functional>
#include <random>

#include <ceres_slam/stereo_camera.h>
#include <ceres_slam/stereo_reprojection_error.h>

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
class OdometryNode {
public:
    OdometryNode() {
        // Read in parameters
        ros::NodeHandle private_nh("~");
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("approximate_sync", do_approximate_sync_, false);
        private_nh.param("num_features", num_features_, 300);
        private_nh.param("match_goodness", match_goodness_, 0.9);

        // Resolve topic names
        ros::NodeHandle nh;

        std::string camera_ns   = nh.resolveName("camera");
        std::string left_ns     = ros::names::clean(camera_ns + "/left");
        std::string right_ns    = ros::names::clean(camera_ns + "/right");

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

        // Subscribe to four input topics
        image_transport::ImageTransport it(nh);
        left_image_sub_.subscribe(it, left_image_topic, 1);
        left_info_sub_.subscribe(nh, left_info_topic, 1);
        right_image_sub_.subscribe(it, right_image_topic, 1);
        right_info_sub_.subscribe(nh, right_info_topic, 1);

        // Synchronize input topics. Optionally do approximate sync.
        if(do_approximate_sync_) {
            approximate_sync_.reset(
                new ApproximateSync(ApproximatePolicy(queue_size_),
                                    left_image_sub_, left_info_sub_,
                                    right_image_sub_, right_info_sub_) );
            approximate_sync_->registerCallback(
                std::bind(&OdometryNode::imageCallback, this,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4) );
        }
        else {
            exact_sync_.reset(
                new ExactSync(ExactPolicy(queue_size_),
                                    left_image_sub_, left_info_sub_,
                                    right_image_sub_, right_info_sub_) );
            exact_sync_->registerCallback(
                std::bind(&OdometryNode::imageCallback, this,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4) );
        }

        // Setup TF pose publisher


    }

    ~OdometryNode() {}

    //! Stereo image callback
    void imageCallback(const sensor_msgs::ImageConstPtr& left_image,
                       const sensor_msgs::CameraInfoConstPtr& left_info,
                       const sensor_msgs::ImageConstPtr& right_image,
                       const sensor_msgs::CameraInfoConstPtr& right_info) {
        // Initialize the camera model if not already done
        if(camera_ == nullptr) {
            camera_ = ceres_slam::StereoCamera::Ptr(
                new ceres_slam::StereoCamera(left_info, right_info));
        }

        // Convert to CV images
        cv_bridge::CvImagePtr left_cv_ptr;
        cv_bridge::CvImagePtr right_cv_ptr;

        try {
            left_cv_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
            right_cv_ptr = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);
        }
        catch(cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Detect features in stereo pair
        cv::ORB detect(num_features_);

        std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat left_descriptors, right_descriptors;
        detect(left_cv_ptr->image, cv::noArray(),
               left_keypoints, left_descriptors);
        detect(right_cv_ptr->image, cv::noArray(),
               right_keypoints, right_descriptors);

        // Find the 2 closest matches for each feature
        cv::BFMatcher matcher(cv::NORM_L2);
        std::vector<std::vector<cv::DMatch>> left_right_matches;
        matcher.knnMatch(left_descriptors, right_descriptors,
                         left_right_matches, 2);

        // Filter for good matches based on goodness ratio
        std::vector<cv::KeyPoint> curr_left_keypoints, curr_right_keypoints;
        cv::Mat curr_left_descriptors, curr_right_descriptors;
        int left_idx, right_idx;
        for(unsigned int i = 0; i < left_right_matches.size(); ++i) {
            if (left_right_matches[i][0].distance <
                        match_goodness_ * left_right_matches[i][1].distance) {
                left_idx = left_right_matches[i][0].trainIdx;
                right_idx = left_right_matches[i][0].queryIdx;

                curr_left_keypoints.push_back(left_keypoints[left_idx]);
                curr_right_keypoints.push_back(right_keypoints[right_idx]);

                curr_left_descriptors.push_back(
                    left_descriptors.row(left_idx));
                curr_right_descriptors.push_back(
                    right_descriptors.row(right_idx));
            }
        }

        // Compute transform from previous camera to current camera
        if(prev_left_keypoints_.size() == 0) {
            T_curr_map_ = Eigen::Affine3d::Identity();
            publishPose();
        }
        else {
            // Find matches between previous stereo pair and current stereo pair
            // We'll just reuse the matcher object from before
            // and match between the left images
            std::vector<std::vector<cv::DMatch>> prev_curr_matches;
            matcher.knnMatch(prev_left_descriptors_, curr_left_descriptors,
                prev_curr_matches, 2);

            // Again, filter for good matches based on goodness ratio
            std::vector<cv::KeyPoint> vo_prev_left_keypoints,
                                      vo_prev_right_keypoints,
                                      vo_curr_left_keypoints,
                                      vo_curr_right_keypoints;
            int prev_idx, curr_idx;
            for(unsigned int i = 0; i < prev_curr_matches.size(); ++i) {
                if (prev_curr_matches[i][0].distance <
                        match_goodness_ * prev_curr_matches[i][1].distance) {
                    prev_idx = prev_curr_matches[i][0].trainIdx;
                    curr_idx = prev_curr_matches[i][0].queryIdx;

                    vo_prev_left_keypoints.push_back(
                        prev_left_keypoints_[prev_idx]);
                    vo_prev_right_keypoints.push_back(
                        prev_right_keypoints_[prev_idx]);
                    vo_curr_left_keypoints.push_back(
                        curr_left_keypoints[curr_idx]);
                    vo_curr_right_keypoints.push_back(
                        curr_right_keypoints[curr_idx]);
                }
            }

            // Triangulate to 3D points
            std::vector<ceres_slam::StereoCamera::Point> prev_pts, curr_pts;
            ceres_slam::StereoCamera::Observation prev_obs, curr_obs;
            for(unsigned int i = 0; i < vo_prev_left_keypoints.size(); ++i) {
                prev_obs << vo_prev_left_keypoints[i].pt.x,
                            vo_prev_left_keypoints[i].pt.y,
                            vo_prev_right_keypoints[i].pt.x,
                            vo_prev_right_keypoints[i].pt.y;
                prev_pts.push_back(camera_->observationToPoint(prev_obs));

                curr_obs << vo_curr_left_keypoints[i].pt.x,
                            vo_curr_left_keypoints[i].pt.y,
                            vo_curr_right_keypoints[i].pt.x,
                            vo_curr_right_keypoints[i].pt.y;
                curr_pts.push_back(camera_->observationToPoint(curr_obs));
            }

            // Filter outliers and generate initial guess with RANSAC
            Eigen::Affine3d T_curr_prev;
            std::vector<unsigned int> inlier_idx =
                doRANSAC(prev_pts, curr_pts, T_curr_prev);

            // TODO: Optimize over inliers with Ceres

            // Update and publish
            T_curr_map_ = T_curr_prev * T_curr_map_;
            publishPose();
        }

        prev_left_keypoints_ = curr_left_keypoints;
        prev_right_keypoints_ = curr_right_keypoints;
        prev_left_descriptors_ = curr_left_descriptors;
        prev_right_descriptors_ = curr_right_descriptors;
    }

    //! RANSAC stereo point cloud alignment
    std::vector<unsigned int> doRANSAC(
                    const std::vector<ceres_slam::StereoCamera::Point>& pts_0,
                    const std::vector<ceres_slam::StereoCamera::Point>& pts_1,
                    Eigen::Affine3d& T_1_0,
                    int num_iters = 600, double thresh = 1e-2) {

        // Uniformly distributed integers in [a,b], NOT [a,b)
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<unsigned int> idx_selector(0, pts_0.size()-1);
        unsigned int rand_idx[3];

        std::vector<ceres_slam::StereoCamera::Point> test_pts_0, test_pts_1;
        std::vector<unsigned int> inlier_idx, best_inlier_idx;

        for(int ransac_iter = 0; ransac_iter < num_iters; ++ransac_iter) {
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
            for(unsigned int i = 0; i < 2; ++i) {
                test_pts_0.push_back(pts_0[rand_idx[i]]);
                test_pts_1.push_back(pts_1[rand_idx[i]]);
            }

            // Compute minimal transformation estimate
            Eigen::Affine3d T_test = alignPointClouds(test_pts_0, test_pts_1);

            // Classify points and get inlier indices
            Eigen::Vector3d error;
            inlier_idx.clear();
            for(unsigned int i = 0; i < pts_0.size(); ++i) {
                error = pts_1[i] - T_test * pts_0[i];
                if(error.squaredNorm() < 2 * thresh) {
                    inlier_idx.push_back(i);
                }
            }

            if(inlier_idx.size() > best_inlier_idx.size()) {
                best_inlier_idx = inlier_idx;
                T_1_0 = T_test;
            }
        }

        return best_inlier_idx;
    }

    Eigen::Affine3d alignPointClouds(
                    const std::vector<ceres_slam::StereoCamera::Point>& pts_0,
                    const std::vector<ceres_slam::StereoCamera::Point>& pts_1) {
        unsigned int n_pts = pts_0.size();

        // Compute the centroids of each cloud
        ceres_slam::StereoCamera::Point p_0 = ceres_slam::StereoCamera::Point::Zero();
        ceres_slam::StereoCamera::Point p_1 = ceres_slam::StereoCamera::Point::Zero();
        for(unsigned int i = 0; i < n_pts; ++i) {
            p_0 += pts_0[i];
            p_1 += pts_1[i];
        }
        p_0 /= n_pts;
        p_1 /= n_pts;

        // Compute W_1_0
        Eigen::Matrix3d W_1_0 = Eigen::Matrix3d::Zero();
        for(unsigned int i = 0; i < pts_0.size(); ++i) {
            W_1_0 += (pts_1[i] - p_1) * (pts_0[i] - p_0).transpose();
        }
        W_1_0 /= n_pts;

        // Compute rotation
        Eigen::Matrix3d C_1_0;

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W_1_0, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d middle = Eigen::Matrix3d::Identity();
        middle(2,2) = svd.matrixU().determinant() * svd.matrixV().determinant();

        C_1_0 = svd.matrixU() * middle * svd.matrixV().transpose();

        // Compute translation
        Eigen::Vector3d r_0_1_0 = -C_1_0.transpose() * p_1 + p_0;

        // Compute final transformation
        Eigen::Affine3d T_1_0;
        T_1_0.linear() = C_1_0;
        T_1_0.translation() = -C_1_0 * r_0_1_0;

        return T_1_0;
    }

    //! Pose publisher
    void publishPose() {
        // Convert to TF pose
        tf::Pose tf_pose;
        tf::poseEigenToTF(T_curr_map_, tf_pose);

        // Publish
        pose_broadcaster_.sendTransform(
            tf::StampedTransform(tf_pose, ros::Time::now(), "map", "camera") );
    }

private:
    // Parameters
    int queue_size_;
    bool do_approximate_sync_;
    int num_features_;
    double match_goodness_;

    // Camera model
    ceres_slam::StereoCamera::Ptr camera_;

    // Camera pose
    Eigen::Affine3d T_curr_map_;

    // Storage for previous frame's keypoints and descriptors
    std::vector<cv::KeyPoint> prev_left_keypoints_, prev_right_keypoints_;
    cv::Mat prev_left_descriptors_, prev_right_descriptors_;

    // Subscribers and setup for synchronization
    image_transport::SubscriberFilter left_image_sub_, right_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;

    typedef message_filters::sync_policies::ExactTime
        <sensor_msgs::Image, sensor_msgs::CameraInfo,
         sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::CameraInfo,
         sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;

    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

    std::shared_ptr<ExactSync> exact_sync_;
    std::shared_ptr<ApproximateSync> approximate_sync_;

    // Publishers
    tf::TransformBroadcaster pose_broadcaster_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_node", ros::init_options::AnonymousName);

    if(ros::names::remap("camera") == "camera") {
    ROS_WARN("'camera' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun ceres_slam odometry_node camera:=xb3");
    }

    OdometryNode odometry_node;

    ros::spin();

    return EXIT_SUCCESS;
}
