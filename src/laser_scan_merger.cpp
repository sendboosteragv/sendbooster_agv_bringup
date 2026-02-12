/*
 * Laser Scan Merger Node
 * Merges front and back LiDAR scans into a single 360° scan
 *
 * Author: 이성민 (roboticsmaster@naver.com)
 * Company: TerraNox
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cmath>
#include <algorithm>

class LaserScanMerger : public rclcpp::Node
{
public:
    LaserScanMerger() : Node("laser_scan_merger")
    {
        // Declare parameters (use_sim_time is auto-declared in ROS2 Humble)
        this->declare_parameter("destination_frame", "base_link");
        this->declare_parameter("scan_destination_topic", "/scan_merged");
        this->declare_parameter("lasers_count", 2);

        destination_frame_ = this->get_parameter("destination_frame").as_string();
        merged_topic_ = this->get_parameter("scan_destination_topic").as_string();

        // Create subscribers with message filters for synchronization
        front_sub_.subscribe(this, "/scan");
        back_sub_.subscribe(this, "/scan2");

        // Approximate time synchronizer with 500ms slop tolerance
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), front_sub_, back_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration(0, 500000000));  // 500ms tolerance
        sync_->registerCallback(std::bind(&LaserScanMerger::scanCallback, this,
            std::placeholders::_1, std::placeholders::_2));

        // Publisher for merged scan
        merged_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            merged_topic_, rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "Laser Scan Merger initialized");
        RCLCPP_INFO(this->get_logger(), "  Front scan: /scan");
        RCLCPP_INFO(this->get_logger(), "  Back scan: /scan2");
        RCLCPP_INFO(this->get_logger(), "  Merged output: %s", merged_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Frame: %s", destination_frame_.c_str());
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> front_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> back_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_pub_;

    std::string destination_frame_;
    std::string merged_topic_;

    void scanCallback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& front_scan,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& back_scan)
    {
        // Create merged scan message
        auto merged_scan = sensor_msgs::msg::LaserScan();

        // Use the timestamp from the front scan
        merged_scan.header.stamp = front_scan->header.stamp;
        merged_scan.header.frame_id = destination_frame_;

        // Full 360° scan
        merged_scan.angle_min = -M_PI;
        merged_scan.angle_max = M_PI;

        // Use front scan's angular resolution
        merged_scan.angle_increment = front_scan->angle_increment;
        merged_scan.time_increment = front_scan->time_increment;
        merged_scan.scan_time = front_scan->scan_time;
        merged_scan.range_min = front_scan->range_min;
        merged_scan.range_max = front_scan->range_max;

        // Calculate total number of readings for 360°
        int total_readings = static_cast<int>(
            std::round((merged_scan.angle_max - merged_scan.angle_min) /
                       merged_scan.angle_increment)) + 1;

        // Initialize with infinity (no reading)
        merged_scan.ranges.resize(total_readings, std::numeric_limits<float>::infinity());
        merged_scan.intensities.resize(total_readings, 0.0f);

        // Merge front scan (angles from -90° to +90° in front LiDAR frame)
        mergeScan(merged_scan, front_scan, 0.0);  // front scan, no rotation offset

        // Merge back scan (angles from -90° to +90° in back LiDAR frame)
        // TF handles the 180° rotation, so we just apply the same offset
        mergeScan(merged_scan, back_scan, M_PI);  // back scan, 180° offset to convert to robot frame

        // Publish merged scan
        merged_pub_->publish(merged_scan);
    }

    void mergeScan(
        sensor_msgs::msg::LaserScan& merged,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
        double angle_offset)
    {
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            // Calculate the angle for this reading in the source scan
            double angle = scan->angle_min + i * scan->angle_increment;

            // Apply rotation offset
            angle += angle_offset;

            // Normalize angle to [-PI, PI]
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;

            // Calculate index in merged scan
            int merged_index = static_cast<int>(
                std::round((angle - merged.angle_min) / merged.angle_increment));

            // Bounds check
            if (merged_index >= 0 && merged_index < static_cast<int>(merged.ranges.size())) {
                float range = scan->ranges[i];

                // Only use valid readings
                if (range >= scan->range_min && range <= scan->range_max) {
                    // Keep the closer reading if there's overlap
                    if (range < merged.ranges[merged_index]) {
                        merged.ranges[merged_index] = range;
                        if (i < scan->intensities.size()) {
                            merged.intensities[merged_index] = scan->intensities[i];
                        }
                    }
                }
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
