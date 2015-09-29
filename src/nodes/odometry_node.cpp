#include <iostream>
#include <cstdlib>
#include <memory>
#include <functional>

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
        std::vector<cv::KeyPoint> good_left_keypoints, good_right_keypoints;
        cv::Mat good_left_descriptors, good_right_descriptors;
        int left_idx, right_idx;
        for(unsigned int i = 0; i < left_right_matches.size(); ++i) {
            if (left_right_matches[i][0].distance <
                        match_goodness_ * left_right_matches[i][1].distance) {
                left_idx = left_right_matches[i][0].trainIdx;
                right_idx = left_right_matches[i][0].queryIdx;

                good_left_keypoints.push_back(left_keypoints[left_idx]);
                good_right_keypoints.push_back(right_keypoints[right_idx]);

                good_left_descriptors.push_back(
                    left_descriptors.row(left_idx));
                good_right_descriptors.push_back(
                    right_descriptors.row(right_idx));
            }
        }

        // Compute transform from previous camera to current camera
        if(prev_left_keypoints.size() == 0) {
            pose_ = Eigen::Affine3d::Identity();
            publishPose();
        }
        else {
            // TODO: Find matches between previous stereo pair and current stereo pair
            // TODO: Reject outliers with RANSAC
            // TODO: Optimize over inliers with Ceres
            publishPose();
        }

        prev_left_keypoints = good_left_keypoints;
        prev_right_keypoints = good_right_keypoints;
        prev_left_descriptors = good_left_descriptors;
        prev_right_descriptors = good_right_descriptors;
    }

    void publishPose() {

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
    Eigen::Affine3d pose_;

    // Storage for previous frame's keypoints and descriptors
    std::vector<cv::KeyPoint> prev_left_keypoints, prev_right_keypoints;
    cv::Mat prev_left_descriptors, prev_right_descriptors;

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
