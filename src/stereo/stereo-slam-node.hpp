#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include <Eigen/Geometry>
#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <mutex>
#include <sophus/se3.hpp>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "Atlas.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"
#include "Tracking.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utility.hpp"

class StereoSlamNode : public rclcpp::Node {
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const std::string& strSettingsFile, const std::string& strDoRectify);

    ~StereoSlamNode();
private:
    using ImageMsg = sensor_msgs::msg::Image;
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    // ---- Image callback (runs on default callback group / tracking thread) ----
    void GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight);

    // ---- Map-data timer (runs on a separate callback group) ----
    void MapDataTimerCallback();

    // ---- Light publish helpers (called from image callback, SAFE) ----
    // Pose and TF only use the stack-returned Sophus::SE3f from TrackStereo(),
    // so they never touch ORB-SLAM3 internals that other threads mutate.
    void publishPoseAndTF(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp);

    // ---- Heavy publish helpers (called from timer, under map mutex) ----
    void publishMapPointsLocked(const rclcpp::Time& stamp);
    void publishKeyFramesLocked(const rclcpp::Time& stamp);

    // ---- Math helpers ----
    Sophus::SE3f orbToRos(const Sophus::SE3f& Twc_orb) const;
    Eigen::Vector3f orbPointToRos(const Eigen::Vector3f& p_orb) const;

    // ---- ORB-SLAM3 ----
    ORB_SLAM3::System* m_SLAM;

    // ---- Rectification (unchanged from upstream zang09) ----
    bool doRectify;
    cv::Mat M1l, M2l, M1r, M2r;
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    // ---- Subscribers ----
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    // ---- Publishers ----
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr keyframes_pub_;

    // ---- Timer (only created when publish_map_data_ is true) ----
    rclcpp::CallbackGroup::SharedPtr map_cb_group_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    // ---- TF ----
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // ---- Parameters ----
    std::string map_frame_id_;
    std::string base_frame_id_;
    std::string camera_frame_id_;
    bool publish_tf_;
    bool publish_map_data_; // Master switch for map points + keyframes
    double map_publish_rate_hz_;
    bool publish_static_base_to_camera_;

    // Fixed rotation: ORB optical -> ROS body
    Eigen::Matrix3f R_ros_orb_;
};

#endif