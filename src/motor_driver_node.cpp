/**
 * @file motor_driver_node.cpp
 * @brief ROS2 Motor Driver Node for MDROBOT Motor Driver
 * Based on MD_controller (https://github.com/jjletsgo/MD_controller)
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "sendbooster_agv_bringup/motor_driver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace sendbooster_agv_bringup
{

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode()
    : Node("motor_driver_node")
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , last_left_position_(0)
    , last_right_position_(0)
    , target_linear_(0.0)
    , target_angular_(0.0)
    , watchdog_triggered_(false)
    , comm_ok_(false)
    {
        // Declare parameters
        declareParameters();

        // Get parameters
        getParameters();

        // Initialize motor driver
        driver_ = std::make_unique<MotorDriver>(
            serial_port_,
            baudrate_,
            mdui_id_,
            mdt_id_,
            motor_id_,
            gear_ratio_,
            poles_
        );

        // Set error callback
        driver_->setErrorCallback(
            std::bind(&MotorDriverNode::errorCallback, this, _1, _2)
        );

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        // Motor status publishers
        left_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/left/rpm", 10);
        right_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/right/rpm", 10);
        left_current_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/left/current", 10);
        right_current_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/right/current", 10);

        // Create subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorDriverNode::cmdVelCallback, this, _1)
        );

        // Create services
        emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/emergency_stop",
            std::bind(&MotorDriverNode::emergencyStopCallback, this, _1, _2)
        );

        reset_position_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_position",
            std::bind(&MotorDriverNode::resetPositionCallback, this, _1, _2)
        );

        reset_alarm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_alarm",
            std::bind(&MotorDriverNode::resetAlarmCallback, this, _1, _2)
        );

        set_torque_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "~/set_torque",
            std::bind(&MotorDriverNode::setTorqueCallback, this, _1, _2)
        );

        // Create timers
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
            std::bind(&MotorDriverNode::controlCallback, this)
        );

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&MotorDriverNode::publishCallback, this)
        );

        diagnostic_timer_ = this->create_wall_timer(
            1s,  // 1Hz diagnostic publishing
            std::bind(&MotorDriverNode::diagnosticCallback, this)
        );

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(watchdog_timeout_ * 1000)),
            std::bind(&MotorDriverNode::watchdogCallback, this)
        );

        // Initialize time
        last_time_ = this->now();
        last_cmd_time_ = this->now();
        last_comm_time_ = this->now();

        // Connect to motor driver
        connectDriver();

        RCLCPP_INFO(this->get_logger(), "Motor driver node initialized");
    }

    ~MotorDriverNode()
    {
        if (driver_) {
            driver_->sendStop();
            driver_->disconnect();
        }
    }

private:
    void declareParameters()
    {
        // Serial communication
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 57600);

        // Motor driver IDs
        this->declare_parameter("mdui_id", 184);
        this->declare_parameter("mdt_id", 183);
        this->declare_parameter("motor_id", 1);

        // Motor parameters
        this->declare_parameter("gear_ratio", 15);
        this->declare_parameter("poles", 10);

        // Robot parameters
        this->declare_parameter("wheel_radius", 0.05);
        this->declare_parameter("wheel_separation", 0.3);

        // Encoder parameters
        this->declare_parameter("encoder_resolution", 6535);

        // Control parameters
        this->declare_parameter("max_rpm", 100);
        this->declare_parameter("control_rate", 50.0);
        this->declare_parameter("publish_rate", 50.0);
        this->declare_parameter("cmd_vel_timeout", 0.5);
        this->declare_parameter("watchdog_timeout", 1.0);

        // Frame IDs
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("base_frame_id", "base_link");

        // TF publishing (disable when using robot_localization EKF)
        this->declare_parameter("publish_tf", true);
    }

    void getParameters()
    {
        // Serial communication
        serial_port_ = this->get_parameter("serial_port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        // Motor driver IDs
        mdui_id_ = static_cast<uint8_t>(this->get_parameter("mdui_id").as_int());
        mdt_id_ = static_cast<uint8_t>(this->get_parameter("mdt_id").as_int());
        motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());

        // Motor parameters
        gear_ratio_ = this->get_parameter("gear_ratio").as_int();
        poles_ = this->get_parameter("poles").as_int();

        // Robot parameters
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();

        // Encoder parameters
        encoder_resolution_ = this->get_parameter("encoder_resolution").as_int();

        // Control parameters
        max_rpm_ = this->get_parameter("max_rpm").as_int();
        control_rate_ = this->get_parameter("control_rate").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();

        // Frame IDs
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();

        // TF publishing
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // Log parameters
        RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);
        RCLCPP_INFO(this->get_logger(), "Wheel radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "Gear ratio: %d", gear_ratio_);
        RCLCPP_INFO(this->get_logger(), "Max RPM: %d", max_rpm_);
    }

    void connectDriver()
    {
        if (driver_->connect()) {
            RCLCPP_INFO(this->get_logger(), "Connected to motor driver on %s", serial_port_.c_str());
            driver_->resetPosition();
            comm_ok_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to motor driver on %s", serial_port_.c_str());
            comm_ok_ = false;
        }
    }

    void errorCallback(MotorError error, const std::string& msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Motor driver error: %s", msg.c_str());

        if (error == MotorError::SERIAL_DISCONNECTED ||
            error == MotorError::SERIAL_WRITE_FAILED ||
            error == MotorError::SERIAL_READ_FAILED) {
            comm_ok_ = false;
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_ = msg->linear.x;
        target_angular_ = msg->angular.z;
        last_cmd_time_ = this->now();
        watchdog_triggered_ = false;
    }

    std::pair<int, int> twistToRpm(double linear, double angular)
    {
        // Differential drive kinematics
        // Angular signs swapped to match physical motor layout
        // (Motor1 = physical right, Motor2 = physical left)
        double v_left = linear + (angular * wheel_separation_ / 2.0);
        double v_right = linear - (angular * wheel_separation_ / 2.0);

        // Convert linear velocity to angular velocity (rad/s)
        double omega_left = v_left / wheel_radius_;
        double omega_right = v_right / wheel_radius_;

        // Convert to RPM
        double rpm_left = omega_left * 60.0 / (2.0 * M_PI);
        double rpm_right = omega_right * 60.0 / (2.0 * M_PI);

        // Clamp to max RPM
        rpm_left = std::clamp(rpm_left, static_cast<double>(-max_rpm_), static_cast<double>(max_rpm_));
        rpm_right = std::clamp(rpm_right, static_cast<double>(-max_rpm_), static_cast<double>(max_rpm_));

        return {static_cast<int>(rpm_left), static_cast<int>(rpm_right)};
    }

    void controlCallback()
    {
        if (!driver_->isConnected()) {
            return;
        }

        // Check for cmd_vel timeout
        auto current_time = this->now();
        double time_since_cmd = (current_time - last_cmd_time_).seconds();

        if (time_since_cmd > cmd_vel_timeout_) {
            target_linear_ = 0.0;
            target_angular_ = 0.0;
        }

        // Convert to RPM and send
        auto [rpm_left, rpm_right] = twistToRpm(target_linear_, target_angular_);
        if (driver_->sendRpm(rpm_left, rpm_right)) {
            last_comm_time_ = current_time;
            comm_ok_ = true;
        }

        // Read response
        if (driver_->readResponse()) {
            last_comm_time_ = current_time;
            comm_ok_ = true;
        }
    }

    double encoderToMeters(int32_t encoder_ticks)
    {
        // wheel_rotations = ticks / (encoder_resolution * gear_ratio)
        double wheel_rotations = static_cast<double>(encoder_ticks) /
                                 (static_cast<double>(encoder_resolution_) * static_cast<double>(gear_ratio_));
        double distance = wheel_rotations * 2.0 * M_PI * wheel_radius_;
        return distance;
    }

    void publishCallback()
    {
        if (!driver_->isConnected()) {
            return;
        }

        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0) {
            return;
        }

        // Get motor states
        const auto& left_state = driver_->getLeftMotor();
        const auto& right_state = driver_->getRightMotor();

        // Publish motor status
        publishMotorStatus(left_state, right_state);

        // Calculate wheel displacements (in meters)
        // Right motor is inverted in sendRpm(), so negate its encoder delta
        double left_delta = encoderToMeters(left_state.position - last_left_position_);
        double right_delta = -encoderToMeters(right_state.position - last_right_position_);

        last_left_position_ = left_state.position;
        last_right_position_ = right_state.position;

        // Calculate odometry using differential drive model
        double delta_s = (left_delta + right_delta) / 2.0;
        double delta_theta = (left_delta - right_delta) / wheel_separation_;

        // Update pose
        x_ += delta_s * std::cos(theta_ + delta_theta / 2.0);
        y_ += delta_s * std::sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;

        // Normalize theta to [-pi, pi]
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        // Calculate velocities
        double linear_vel = (dt > 0) ? delta_s / dt : 0.0;
        double angular_vel = (dt > 0) ? delta_theta / dt : 0.0;

        // Publish odometry
        publishOdometry(current_time, linear_vel, angular_vel);

        // Publish TF (disabled when EKF handles it)
        if (publish_tf_) {
            publishTF(current_time);
        }
    }

    void publishMotorStatus(const MotorState& left, const MotorState& right)
    {
        std_msgs::msg::Float32 msg;

        msg.data = static_cast<float>(left.rpm);
        left_rpm_pub_->publish(msg);

        msg.data = static_cast<float>(right.rpm);
        right_rpm_pub_->publish(msg);

        msg.data = static_cast<float>(left.current) / 1000.0f;  // mA to A
        left_current_pub_->publish(msg);

        msg.data = static_cast<float>(right.current) / 1000.0f;
        right_current_pub_->publish(msg);
    }

    void publishOdometry(const rclcpp::Time& stamp, double linear_vel, double angular_vel)
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        // Position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Orientation (quaternion from yaw)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Velocity
        odom.twist.twist.linear.x = linear_vel;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angular_vel;

        // Covariance (simple diagonal)
        odom.pose.covariance[0] = 0.01;   // x
        odom.pose.covariance[7] = 0.01;   // y
        odom.pose.covariance[35] = 0.01;  // yaw

        odom.twist.covariance[0] = 0.01;   // vx
        odom.twist.covariance[35] = 0.01;  // vyaw

        odom_pub_->publish(odom);
    }

    void publishTF(const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = odom_frame_id_;
        t.child_frame_id = base_frame_id_;

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    void diagnosticCallback()
    {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();

        // Motor driver status
        diagnostic_msgs::msg::DiagnosticStatus driver_status;
        driver_status.name = "Motor Driver";
        driver_status.hardware_id = serial_port_;

        if (!driver_->isConnected()) {
            driver_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            driver_status.message = "Disconnected";
        } else if (!comm_ok_) {
            driver_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            driver_status.message = "Communication issues";
        } else {
            driver_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            driver_status.message = "OK";
        }

        // Add key-value pairs
        diagnostic_msgs::msg::KeyValue kv;

        kv.key = "Port";
        kv.value = serial_port_;
        driver_status.values.push_back(kv);

        kv.key = "Baudrate";
        kv.value = std::to_string(baudrate_);
        driver_status.values.push_back(kv);

        kv.key = "TX Count";
        kv.value = std::to_string(driver_->getTxCount());
        driver_status.values.push_back(kv);

        kv.key = "RX Count";
        kv.value = std::to_string(driver_->getRxCount());
        driver_status.values.push_back(kv);

        kv.key = "Error Count";
        kv.value = std::to_string(driver_->getErrorCount());
        driver_status.values.push_back(kv);

        diag_array.status.push_back(driver_status);

        // Left motor status
        const auto& left = driver_->getLeftMotor();
        diagnostic_msgs::msg::DiagnosticStatus left_status;
        left_status.name = "Left Motor";
        left_status.hardware_id = "motor_1";

        if (left.hasError()) {
            left_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            left_status.message = "Motor error detected";
        } else {
            left_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            left_status.message = "OK";
        }

        kv.key = "RPM";
        kv.value = std::to_string(left.rpm);
        left_status.values.push_back(kv);

        kv.key = "Current (mA)";
        kv.value = std::to_string(left.current);
        left_status.values.push_back(kv);

        kv.key = "Position";
        kv.value = std::to_string(left.position);
        left_status.values.push_back(kv);

        kv.key = "Temperature";
        kv.value = std::to_string(left.driver_temp);
        left_status.values.push_back(kv);

        diag_array.status.push_back(left_status);

        // Right motor status
        const auto& right = driver_->getRightMotor();
        diagnostic_msgs::msg::DiagnosticStatus right_status;
        right_status.name = "Right Motor";
        right_status.hardware_id = "motor_2";

        if (right.hasError()) {
            right_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            right_status.message = "Motor error detected";
        } else {
            right_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            right_status.message = "OK";
        }

        kv.key = "RPM";
        kv.value = std::to_string(right.rpm);
        right_status.values.push_back(kv);

        kv.key = "Current (mA)";
        kv.value = std::to_string(right.current);
        right_status.values.push_back(kv);

        kv.key = "Position";
        kv.value = std::to_string(right.position);
        right_status.values.push_back(kv);

        kv.key = "Temperature";
        kv.value = std::to_string(right.driver_temp);
        right_status.values.push_back(kv);

        diag_array.status.push_back(right_status);

        diagnostic_pub_->publish(diag_array);
    }

    void watchdogCallback()
    {
        auto current_time = this->now();
        double time_since_comm = (current_time - last_comm_time_).seconds();

        if (time_since_comm > watchdog_timeout_ && !watchdog_triggered_) {
            RCLCPP_WARN(this->get_logger(), "Watchdog triggered - no communication for %.1f seconds", time_since_comm);
            watchdog_triggered_ = true;

            // Stop motors for safety
            if (driver_->isConnected()) {
                driver_->sendStop();
            }

            comm_ok_ = false;
        }
    }

    // Service callbacks
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_WARN(this->get_logger(), "Emergency stop requested!");

        if (driver_->emergencyStop()) {
            response->success = true;
            response->message = "Emergency stop executed";
        } else {
            response->success = false;
            response->message = "Failed to execute emergency stop";
        }
    }

    void resetPositionCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Reset position requested");

        if (driver_->resetPosition()) {
            // Also reset odometry
            x_ = 0.0;
            y_ = 0.0;
            theta_ = 0.0;
            last_left_position_ = 0;
            last_right_position_ = 0;

            response->success = true;
            response->message = "Position reset successful";
        } else {
            response->success = false;
            response->message = "Failed to reset position";
        }
    }

    void resetAlarmCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Reset alarm requested");

        if (driver_->resetAlarm()) {
            response->success = true;
            response->message = "Alarm reset successful";
        } else {
            response->success = false;
            response->message = "Failed to reset alarm";
        }
    }

    void setTorqueCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Set torque %s requested", request->data ? "ON" : "OFF");

        if (driver_->setTorque(request->data)) {
            response->success = true;
            response->message = request->data ? "Torque enabled" : "Torque disabled";
        } else {
            response->success = false;
            response->message = "Failed to set torque";
        }
    }

private:
    // Motor driver
    std::unique_ptr<MotorDriver> driver_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_current_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_position_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_alarm_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Parameters - Serial
    std::string serial_port_;
    int baudrate_;
    uint8_t mdui_id_;
    uint8_t mdt_id_;
    uint8_t motor_id_;

    // Parameters - Motor
    int gear_ratio_;
    int poles_;

    // Parameters - Robot
    double wheel_radius_;
    double wheel_separation_;

    // Parameters - Encoder
    int encoder_resolution_;

    // Parameters - Control
    int max_rpm_;
    double control_rate_;
    double publish_rate_;
    double cmd_vel_timeout_;
    double watchdog_timeout_;

    // Parameters - Frames
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool publish_tf_;

    // Odometry state
    double x_;
    double y_;
    double theta_;
    int32_t last_left_position_;
    int32_t last_right_position_;
    rclcpp::Time last_time_;

    // Control state
    double target_linear_;
    double target_angular_;
    rclcpp::Time last_cmd_time_;

    // Watchdog
    rclcpp::Time last_comm_time_;
    bool watchdog_triggered_;
    bool comm_ok_;
};

}  // namespace sendbooster_agv_bringup

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sendbooster_agv_bringup::MotorDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
