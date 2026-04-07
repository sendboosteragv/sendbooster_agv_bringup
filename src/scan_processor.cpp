/**
 * @file scan_processor.cpp
 * @brief Combined scan angle filter + merger using TF for coordinate transforms.
 *
 * Pipeline:
 *   /scan_raw_front ─┐
 *                     ├─ [ScanProcessor] ─→ /scan_merged (frame: base_link)
 *   /scan_raw_back  ─┘
 *
 * Uses TF to transform each LiDAR ray into base_link frame,
 * so URDF-defined LiDAR positions and orientations are automatically applied.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

class ScanProcessor : public rclcpp::Node
{
public:
    ScanProcessor() : Node("scan_processor")
    {
        this->declare_parameter("num_lidars", 2);
        this->declare_parameter("destination_frame", "base_link");
        this->declare_parameter("max_scan_age", 1.0);
        this->declare_parameter("front_angle_min", -M_PI_2);
        this->declare_parameter("front_angle_max", M_PI_2);
        this->declare_parameter("front_invert", true);
        this->declare_parameter("back_angle_min", -M_PI_2);
        this->declare_parameter("back_angle_max", M_PI_2);
        this->declare_parameter("back_invert", true);
        this->declare_parameter("range_max", 6.0);

        num_lidars_ = this->get_parameter("num_lidars").as_int();
        destination_frame_ = this->get_parameter("destination_frame").as_string();
        max_scan_age_ = this->get_parameter("max_scan_age").as_double();
        front_angle_min_ = this->get_parameter("front_angle_min").as_double();
        front_angle_max_ = this->get_parameter("front_angle_max").as_double();
        front_invert_ = this->get_parameter("front_invert").as_bool();
        back_angle_min_ = this->get_parameter("back_angle_min").as_double();
        back_angle_max_ = this->get_parameter("back_angle_max").as_double();
        back_invert_ = this->get_parameter("back_invert").as_bool();
        range_max_override_ = this->get_parameter("range_max").as_double();

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto sensor_qos = rclcpp::SensorDataQoS();

        merged_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan_merged", sensor_qos);

        front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw_front", sensor_qos,
            std::bind(&ScanProcessor::frontCallback, this, std::placeholders::_1));

        if (num_lidars_ >= 2) {
            back_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan_raw_back", sensor_qos,
                std::bind(&ScanProcessor::backCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(),
            "ScanProcessor: %d LiDAR(s), frame=%s, TF-based merge",
            num_lidars_, destination_frame_.c_str());
    }

private:
    void frontCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
    {
        if (isScanStale(msg)) return;

        initMergedScan(msg->header.stamp, msg->angle_increment,
                       msg->range_min, msg->range_max,
                       msg->time_increment, msg->scan_time);

        // Merge front using TF
        double front_yaw = getYawFromTF(msg->header.frame_id);
        mergeScanInto(merged_scan_, msg,
                      front_angle_min_, front_angle_max_, front_invert_, front_yaw);

        // Merge latest back using TF
        if (num_lidars_ >= 2) {
            std::lock_guard<std::mutex> lock(back_mutex_);
            if (latest_back_scan_) {
                double back_yaw = getYawFromTF(latest_back_scan_->header.frame_id);
                mergeScanInto(merged_scan_, latest_back_scan_,
                              back_angle_min_, back_angle_max_, back_invert_, back_yaw);
            }
        }

        merged_pub_->publish(merged_scan_);
    }

    void backCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
    {
        if (isScanStale(msg)) return;
        std::lock_guard<std::mutex> lock(back_mutex_);
        latest_back_scan_ = msg;
    }

    double getYawFromTF(const std::string& source_frame) const
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                destination_frame_, source_frame, tf2::TimePointZero);
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return yaw;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(),
                "TF lookup failed for %s: %s", source_frame.c_str(), ex.what());
            return 0.0;
        }
    }

    bool isScanStale(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) const
    {
        double age = (this->now() - rclcpp::Time(msg->header.stamp)).seconds();
        return age > max_scan_age_;
    }

    void initMergedScan(
        const builtin_interfaces::msg::Time& stamp,
        float angle_inc, float range_min, float range_max,
        float time_inc, float scan_time)
    {
        merged_scan_.header.stamp = stamp;
        merged_scan_.header.frame_id = destination_frame_;
        merged_scan_.angle_min = static_cast<float>(-M_PI);
        merged_scan_.angle_max = static_cast<float>(M_PI);
        merged_scan_.angle_increment = angle_inc;
        merged_scan_.time_increment = time_inc;
        merged_scan_.scan_time = scan_time;
        merged_scan_.range_min = range_min;
        merged_scan_.range_max = std::min(range_max, static_cast<float>(range_max_override_));

        const size_t total = static_cast<size_t>(
            std::round(2.0 * M_PI / angle_inc)) + 1;

        if (merged_scan_.ranges.size() != total) {
            merged_scan_.ranges.resize(total);
            merged_scan_.intensities.resize(total);
        }
        std::fill(merged_scan_.ranges.begin(), merged_scan_.ranges.end(),
                  std::numeric_limits<float>::infinity());
        std::fill(merged_scan_.intensities.begin(), merged_scan_.intensities.end(), 0.0f);
    }

    void mergeScanInto(
        sensor_msgs::msg::LaserScan& merged,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
        double filt_min, double filt_max, bool invert, double yaw_offset) const
    {
        const int merged_size = static_cast<int>(merged.ranges.size());
        const float r_min = scan->range_min;
        const float r_max = std::min(scan->range_max, static_cast<float>(range_max_override_));
        const bool has_intensities = !scan->intensities.empty();

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double local_angle = scan->angle_min + i * scan->angle_increment;

            // Angle filter in local LiDAR frame
            const bool in_range = (local_angle >= filt_min && local_angle <= filt_max);
            if (invert && in_range) continue;
            if (!invert && !in_range) continue;

            const float range = scan->ranges[i];
            if (range < r_min || range > r_max) continue;

            // Transform angle to base_link frame using TF yaw
            double angle = local_angle + yaw_offset;

            // Normalize to [-PI, PI]
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;

            const int idx = static_cast<int>(
                std::round((angle - static_cast<double>(merged.angle_min)) /
                           merged.angle_increment));

            if (idx >= 0 && idx < merged_size && range < merged.ranges[idx]) {
                merged.ranges[idx] = range;
                if (has_intensities) {
                    merged.intensities[idx] = scan->intensities[i];
                }
            }
        }
    }

    // Members
    int num_lidars_;
    std::string destination_frame_;
    double max_scan_age_;
    double front_angle_min_, front_angle_max_;
    bool front_invert_;
    double back_angle_min_, back_angle_max_;
    bool back_invert_;
    double range_max_override_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_back_scan_;
    sensor_msgs::msg::LaserScan merged_scan_;
    std::mutex back_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
