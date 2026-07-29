#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "System.h"

#include <queue>
#include <mutex>
#include <thread>
#include <chrono>

using ImageMsg = sensor_msgs::msg::Image;

class SlamNode : public rclcpp::Node
{
public:
    SlamNode(ORB_SLAM3::System* pSLAM)
    : Node("slam_node"), SLAM_(pSLAM)
    {
        sub_img_left_ = this->create_subscription<ImageMsg>(
            "/camera/left/image_raw", 100,
            std::bind(&SlamNode::GrabImageLeft, this, std::placeholders::_1));

        sub_img_right_ = this->create_subscription<ImageMsg>(
            "/camera/right/image_raw", 100,
            std::bind(&SlamNode::GrabImageRight, this, std::placeholders::_1));

        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orbslam3/pose", 10);

        pub_map_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/slam/map_points", 10);

        sync_thread_ = new std::thread(&SlamNode::SyncStereo, this);

        RCLCPP_INFO(this->get_logger(), "SlamNode ready");
    }

    ~SlamNode()
    {
        sync_thread_->join();
        delete sync_thread_;
        SLAM_->Shutdown();
        SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

private:
    //----------------------------------------------------------------------------------------
    // Callbacks
    //----------------------------------------------------------------------------------------
    void GrabImageLeft(const ImageMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buf_mutex_left_);
        if (!img_left_buf_.empty()) img_left_buf_.pop();
        img_left_buf_.push(msg);
    }

    void GrabImageRight(const ImageMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buf_mutex_right_);
        if (!img_right_buf_.empty()) img_right_buf_.pop();
        img_right_buf_.push(msg);
    }

    //----------------------------------------------------------------------------------------
    // Helpers
    //----------------------------------------------------------------------------------------
    double StampToSec(const builtin_interfaces::msg::Time& stamp)
    {
        return stamp.sec + stamp.nanosec * 1e-9;
    }

    cv::Mat GetImage(const ImageMsg::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        return cv_ptr->image.clone();
    }

    //----------------------------------------------------------------------------------------
    // Publish pose (raw ORB-SLAM3 frame — frame conversion done in slam_bridge.py)
    //----------------------------------------------------------------------------------------
    void PublishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
    {
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f    t = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp    = stamp;
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = t.x();
        pose_msg.pose.position.y = t.y();
        pose_msg.pose.position.z = t.z();

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pub_pose_->publish(pose_msg);
    }

    //----------------------------------------------------------------------------------------
    // Publish map points (converted to ROS frame for RViz2 visualization)
    // ORB-SLAM3 frame: X-right, Y-down, Z-forward
    // ROS frame:       X-forward, Y-left, Z-up
    // Conversion: x_ros = z_orb, y_ros = -x_orb, z_ros = -y_orb
    //----------------------------------------------------------------------------------------
    void PublishMapPoints(const rclcpp::Time& stamp)
    {
        std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM_->GetTrackedMapPoints();

        if (map_points.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp    = stamp;
        cloud_msg.header.frame_id = "map";
        cloud_msg.height          = 1;
        cloud_msg.width           = map_points.size();
        cloud_msg.is_dense        = false;

        sensor_msgs::msg::PointField field_x, field_y, field_z;
        field_x.name     = "x"; field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count    = 1;
        field_y.name     = "y"; field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count    = 1;
        field_z.name     = "z"; field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count    = 1;

        cloud_msg.fields     = {field_x, field_y, field_z};
        cloud_msg.point_step = 12;
        cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.data.resize(cloud_msg.row_step);

        size_t idx = 0;
        for (auto* mp : map_points)
        {
            if (!mp || mp->isBad()) continue;
            Eigen::Matrix<float,3,1> pos = mp->GetWorldPos();

            // Convert ORB-SLAM3 frame -> ROS frame
            float rx =  pos(2);
            float ry = -pos(0);
            float rz = -pos(1);

            memcpy(&cloud_msg.data[idx + 0], &rx, sizeof(float));
            memcpy(&cloud_msg.data[idx + 4], &ry, sizeof(float));
            memcpy(&cloud_msg.data[idx + 8], &rz, sizeof(float));
            idx += cloud_msg.point_step;
        }

        pub_map_points_->publish(cloud_msg);
    }

    //----------------------------------------------------------------------------------------
    // Sync thread: align stereo frames and call TrackStereo (pure stereo, no IMU)
    //----------------------------------------------------------------------------------------
    void SyncStereo()
    {
        const double maxTimeDiff = 0.15;
        int loop_count = 0;

        while (rclcpp::ok())
        {
            loop_count++;

            if (img_left_buf_.empty() || img_right_buf_.empty())
            {
                if (loop_count % 2000 == 0)
                    RCLCPP_INFO(this->get_logger(), "[SYNC] Waiting for images — left:%zu right:%zu",
                        img_left_buf_.size(), img_right_buf_.size());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            double tImLeft, tImRight;

            {
                std::lock_guard<std::mutex> lock(buf_mutex_left_);
                tImLeft = StampToSec(img_left_buf_.front()->header.stamp);
            }
            {
                std::lock_guard<std::mutex> lock(buf_mutex_right_);
                tImRight = StampToSec(img_right_buf_.front()->header.stamp);
            }

            // Drop stale frames to sync left/right
            {
                std::lock_guard<std::mutex> lock(buf_mutex_right_);
                while ((tImLeft - tImRight) > maxTimeDiff && img_right_buf_.size() > 1)
                {
                    img_right_buf_.pop();
                    tImRight = StampToSec(img_right_buf_.front()->header.stamp);
                }
            }
            {
                std::lock_guard<std::mutex> lock(buf_mutex_left_);
                while ((tImRight - tImLeft) > maxTimeDiff && img_left_buf_.size() > 1)
                {
                    img_left_buf_.pop();
                    tImLeft = StampToSec(img_left_buf_.front()->header.stamp);
                }
            }

            if (std::abs(tImLeft - tImRight) > maxTimeDiff)
            {
                RCLCPP_WARN(this->get_logger(), "[SYNC] Stereo time diff too large: %.4f s (left=%.3f right=%.3f)",
                    std::abs(tImLeft - tImRight), tImLeft, tImRight);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Grab images
            cv::Mat imLeft, imRight;
            rclcpp::Time img_stamp;
            {
                std::lock_guard<std::mutex> lock(buf_mutex_left_);
                img_stamp = img_left_buf_.front()->header.stamp;
                imLeft    = GetImage(img_left_buf_.front());
                img_left_buf_.pop();
            }
            {
                std::lock_guard<std::mutex> lock(buf_mutex_right_);
                imRight = GetImage(img_right_buf_.front());
                img_right_buf_.pop();
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[SYNC] Calling TrackStereo — t=%.3f stereo_diff=%.4f s",
                tImLeft, std::abs(tImLeft - tImRight));

            // Track
            Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft);

            int tracking_state = SLAM_->GetTrackingState();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[SYNC] TrackStereo done — state=%d (2=OK, 3=LOST, 1=NOT_INIT)",
                tracking_state);

            if (tracking_state == 2)
            {
                PublishPose(Tcw, img_stamp);
                PublishMapPoints(img_stamp);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    //----------------------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------------------
    ORB_SLAM3::System* SLAM_;

    rclcpp::Subscription<ImageMsg>::SharedPtr  sub_img_left_;
    rclcpp::Subscription<ImageMsg>::SharedPtr  sub_img_right_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_map_points_;

    std::queue<ImageMsg::SharedPtr> img_left_buf_, img_right_buf_;

    std::mutex buf_mutex_left_, buf_mutex_right_;

    std::thread* sync_thread_;
};

//--------------------------------------------------------------------------------------------
// main
//--------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        std::cerr << "Usage: slam_node path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2],
                        ORB_SLAM3::System::STEREO,
                        true);

    auto node = std::make_shared<SlamNode>(&SLAM);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}