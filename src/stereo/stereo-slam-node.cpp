#include "stereo-slam-node.hpp"

#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const std::string& strSettingsFile,
                               const std::string& strDoRectify)
    : Node("orbslam3_stereo"), m_SLAM(pSLAM)
{
    // ---- Parse rectify flag ----
    {
        std::stringstream ss(strDoRectify);
        ss >> std::boolalpha >> doRectify;
    }

    // ---- Declare parameters ----
    map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    camera_frame_id_ = this->declare_parameter<std::string>("camera_frame_id", "camera_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    publish_map_data_ = this->declare_parameter<bool>("publish_map_data", false);
    map_publish_rate_hz_ = this->declare_parameter<double>("map_publish_rate_hz", 2.0);
    publish_static_base_to_camera_ = this->declare_parameter<bool>("publish_static_base_to_camera", true);

    if (map_publish_rate_hz_ <= 0.0)
    {
        map_publish_rate_hz_ = 2.0;
    }

    // ---- Build R_ros_orb_ ----
    // ORB-SLAM camera optical: x_right, y_down, z_forward
    // ROS body / map:          x_forward, y_left, z_up
    // p_ros = R_ros_orb * p_orb
    //   x_ros =  z_orb
    //   y_ros = -x_orb
    //   z_ros = -y_orb
    R_ros_orb_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    // ---- Stereo rectification (unchanged from upstream zang09) ----
    if (doRectify)
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            RCLCPP_FATAL(this->get_logger(), "ERROR: Wrong path to settings");
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty()
            || D_r.empty() || rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            RCLCPP_FATAL(this->get_logger(), "ERROR: Calibration parameters to rectify stereo are missing!");
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F,
                                    M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F,
                                    M1r, M2r);
    }

    // ---- Image subscribers (synchronized) ----
    // Use the raw rclcpp::Node* overload; the upstream zang09 trick of
    // wrapping `this` in a shared_ptr<Node> causes a double-free at shutdown.
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/right");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    // ---- Publishers (always create pose; map/keyframes only if enabled) ----
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orb_slam3/pose", 10);

    if (publish_map_data_)
    {
        map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/map_points", 1);
        keyframes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orb_slam3/keyframes", 1);
    }

    // ---- TF ----
    if (publish_tf_)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    if (publish_static_base_to_camera_)
    {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = base_frame_id_;
        t.child_frame_id = camera_frame_id_;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(t);
    }

    // ---- Map-data timer on its own callback group (only if enabled) ----
    if (publish_map_data_)
    {
        map_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        const auto period = std::chrono::duration<double>(1.0 / map_publish_rate_hz_);
        map_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                             std::bind(&StereoSlamNode::MapDataTimerCallback, this), map_cb_group_);
    }

    RCLCPP_INFO(this->get_logger(),
                "orbslam3_stereo started. doRectify=%d publish_tf=%d publish_map_data=%d rate=%.1f Hz", (int)doRectify,
                (int)publish_tf_, (int)publish_map_data_, map_publish_rate_hz_);
}

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------
StereoSlamNode::~StereoSlamNode()
{
    // Do NOT call m_SLAM->Shutdown() here. stereo.cpp's main() owns the
    // ORB_SLAM3::System on the stack and will call Shutdown() in the right
    // order after rclcpp::spin() returns. Calling it here causes threads to
    // be torn down twice and produces a double-free at process exit.
    RCLCPP_INFO(this->get_logger(), "StereoSlamNode destroyed.");
}

// ---------------------------------------------------------------------------
// Image callback
// ---------------------------------------------------------------------------
void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge L exception: %s", e.what());
        return;
    }
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge R exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw;
    const double t = Utility::StampToSec(msgLeft->header.stamp);

    if (doRectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        Tcw = m_SLAM->TrackStereo(imLeft, imRight, t);
    }
    else
    {
        Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, t);
    }

    // Gate on tracking state. 2 == OK in ORB-SLAM3 (eTrackingState::OK).
    // This mirrors what the original zang09 wrapper does implicitly: if
    // tracking failed, Tcw is an identity (or undefined) pose and we should
    // not publish it as if it were valid.
    if (m_SLAM->GetTrackingState() != 2)
    {
        return;
    }

    publishPoseAndTF(Tcw, msgLeft->header.stamp);
}

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------
Sophus::SE3f StereoSlamNode::orbToRos(const Sophus::SE3f& Twc_orb) const
{
    const Eigen::Matrix3f& R = R_ros_orb_;
    const Eigen::Matrix3f Rt = R.transpose();
    const Eigen::Matrix3f rot_ros = R * Twc_orb.rotationMatrix() * Rt;
    const Eigen::Vector3f t_ros = R * Twc_orb.translation();
    return Sophus::SE3f(rot_ros, t_ros);
}

Eigen::Vector3f StereoSlamNode::orbPointToRos(const Eigen::Vector3f& p_orb) const
{ return R_ros_orb_ * p_orb; }

// ---------------------------------------------------------------------------
// Light publishers — SAFE, only use stack-local Sophus pose
// ---------------------------------------------------------------------------
void StereoSlamNode::publishPoseAndTF(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
{
    // Tcw: world -> camera. We want camera-in-world:
    const Sophus::SE3f Twc_orb = Tcw.inverse();
    const Sophus::SE3f Twc_ros = orbToRos(Twc_orb);

    const Eigen::Quaternionf q(Twc_ros.rotationMatrix());
    const Eigen::Vector3f tr = Twc_ros.translation();

    // --- PoseStamped ---
    if (pose_pub_->get_subscription_count() > 0)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_id_;
        msg.pose.position.x = tr.x();
        msg.pose.position.y = tr.y();
        msg.pose.position.z = tr.z();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        pose_pub_->publish(msg);
    }

    // --- TF map -> base_link ---
    if (publish_tf_ && tf_broadcaster_)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = map_frame_id_;
        tf_msg.child_frame_id = base_frame_id_;
        tf_msg.transform.translation.x = tr.x();
        tf_msg.transform.translation.y = tr.y();
        tf_msg.transform.translation.z = tr.z();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

// ---------------------------------------------------------------------------
// Heavy publishers — run from the map-data timer, under Map::mMutexMapUpdate
// ---------------------------------------------------------------------------
// Helper: pack a vector of XYZ floats into a PointCloud2 message.
static sensor_msgs::msg::PointCloud2 makeXyzCloud(const std::vector<Eigen::Vector3f>& pts, const std::string& frame_id,
                                                  const rclcpp::Time& stamp)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;

    cloud.fields.resize(3);
    const char* names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
    {
        cloud.fields[i].name = names[i];
        cloud.fields[i].offset = i * 4;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }
    cloud.data.resize(cloud.row_step);
    float* dst = reinterpret_cast<float*>(cloud.data.data());
    for (const auto& p: pts)
    {
        *dst++ = p.x();
        *dst++ = p.y();
        *dst++ = p.z();
    }
    return cloud;
}

void StereoSlamNode::MapDataTimerCallback()
{
    if (!publish_map_data_)
    {
        return;
    }

    // Requires the one-line patch to System.h:
    //   Atlas* GetAtlas() { return mpAtlas; }
    ORB_SLAM3::Atlas* atlas = m_SLAM->GetAtlas();
    if (atlas == nullptr)
    {
        return;
    }

    ORB_SLAM3::Map* current = atlas->GetCurrentMap();
    if (current == nullptr)
    {
        return;
    }

    // CRITICAL: hold Map::mMutexMapUpdate while reading keyframes/points.
    // LocalMapping and LoopClosing both lock this when mutating the map,
    // so holding it here prevents bad-pointer dereferences and races.
    // Keep the critical section as short as possible (just copy out the
    // data we need, then release before doing any publishing).
    std::vector<Eigen::Vector3f> mp_pts;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> kf_poses;

    {
        std::unique_lock<std::mutex> lock(current->mMutexMapUpdate);

        if (map_points_pub_ && map_points_pub_->get_subscription_count() > 0)
        {
            const std::vector<ORB_SLAM3::MapPoint*> all = current->GetAllMapPoints();
            mp_pts.reserve(all.size());
            for (auto* mp: all)
            {
                if (mp == nullptr || mp->isBad())
                {
                    continue;
                }
                mp_pts.push_back(orbPointToRos(mp->GetWorldPos()));
            }
        }

        if (keyframes_pub_ && keyframes_pub_->get_subscription_count() > 0)
        {
            const std::vector<ORB_SLAM3::KeyFrame*> kfs = current->GetAllKeyFrames();
            kf_poses.reserve(kfs.size());
            for (auto* kf: kfs)
            {
                if (kf == nullptr || kf->isBad())
                {
                    continue;
                }
                const Sophus::SE3f Twc_orb = kf->GetPoseInverse();
                const Sophus::SE3f Twc_ros = orbToRos(Twc_orb);
                kf_poses.emplace_back(Twc_ros.translation(), Eigen::Quaternionf(Twc_ros.rotationMatrix()));
            }
        }
    } // <-- mMutexMapUpdate released here

    const auto stamp = this->get_clock()->now();

    if (map_points_pub_ && !mp_pts.empty())
    {
        auto cloud = makeXyzCloud(mp_pts, map_frame_id_, stamp);
        map_points_pub_->publish(cloud);
    }

    if (keyframes_pub_ && !kf_poses.empty())
    {
        geometry_msgs::msg::PoseArray pa;
        pa.header.stamp = stamp;
        pa.header.frame_id = map_frame_id_;
        pa.poses.reserve(kf_poses.size());
        for (const auto& [t, q]: kf_poses)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = t.x();
            p.position.y = t.y();
            p.position.z = t.z();
            p.orientation.x = q.x();
            p.orientation.y = q.y();
            p.orientation.z = q.z();
            p.orientation.w = q.w();
            pa.poses.push_back(p);
        }
        keyframes_pub_->publish(pa);
    }
}