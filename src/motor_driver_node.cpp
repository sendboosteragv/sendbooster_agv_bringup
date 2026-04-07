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
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
    , imu_yaw_(0.0)
    , imu_yaw_offset_(0.0)
    , imu_initialized_(false)
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
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        // Motor status publishers
        left_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/left/rpm", 10);
        right_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/right/rpm", 10);
        left_current_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/left/current", 10);
        right_current_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/right/current", 10);

        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorDriverNode::cmdVelCallback, this, _1)
        );

        // IMU subscriber (TurtleBot3 style: use IMU yaw directly for heading)
        if (use_imu_) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 10,
                std::bind(&MotorDriverNode::imuCallback, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "IMU fusion enabled (direct yaw mode)");
        }

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

        // Single unified timer: read → publish → send (no separate publish timer)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
            std::bind(&MotorDriverNode::controlCallback, this)
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
        this->declare_parameter("serial_port", "/dev/motor_driver");
        this->declare_parameter("baudrate", 19200);

        // Motor driver IDs
        this->declare_parameter("mdui_id", 184);
        this->declare_parameter("mdt_id", 183);
        this->declare_parameter("motor_id", 1);

        // Motor parameters
        this->declare_parameter("gear_ratio", 10);
        this->declare_parameter("poles", 10);

        // Robot parameters
        this->declare_parameter("wheel_radius", 0.0965);
        this->declare_parameter("wheel_separation", 0.37);

        // Encoder parameters
        this->declare_parameter("encoder_resolution", 6535);

        // Control parameters
        this->declare_parameter("max_rpm", 100);
        this->declare_parameter("control_rate", 10.0);
        this->declare_parameter("cmd_vel_timeout", 1.0);
        this->declare_parameter("watchdog_timeout", 1.0);

        // Frame IDs
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("base_frame_id", "base_footprint");

        // TF publishing (disable when using robot_localization EKF)
        this->declare_parameter("publish_tf", true);

        // Odometry source: false = command-based, true = encoder-based
        this->declare_parameter("use_encoder_odom", false);

        // IMU fusion: use IMU yaw directly for heading (TurtleBot3 style)
        this->declare_parameter("use_imu", false);
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
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();

        // Frame IDs
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();

        // TF publishing
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // Odometry source
        use_encoder_odom_ = this->get_parameter("use_encoder_odom").as_bool();

        // IMU fusion
        use_imu_ = this->get_parameter("use_imu").as_bool();

        // Log parameters
        RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);
        RCLCPP_INFO(this->get_logger(), "Wheel radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "Gear ratio: %d", gear_ratio_);
        RCLCPP_INFO(this->get_logger(), "Max RPM: %d", max_rpm_);
        RCLCPP_INFO(this->get_logger(), "Odometry source: %s", use_encoder_odom_ ? "encoder" : "command");
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

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract yaw from IMU quaternion
        double siny_cosp = 2.0 * (msg->orientation.w * msg->orientation.z +
                                   msg->orientation.x * msg->orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->orientation.y * msg->orientation.y +
                                          msg->orientation.z * msg->orientation.z);
        double raw_yaw = std::atan2(siny_cosp, cosy_cosp);

        // Store initial yaw offset (AHRS gives absolute yaw relative to magnetic north)
        if (!imu_initialized_) {
            imu_yaw_offset_ = raw_yaw;
            imu_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU initialized: yaw offset = %.2f deg",
                        imu_yaw_offset_ * 180.0 / M_PI);
        }

        // Subtract offset so odom starts at yaw=0
        imu_yaw_ = raw_yaw - imu_yaw_offset_;
        // Normalize to [-pi, pi]
        imu_yaw_ = std::atan2(std::sin(imu_yaw_), std::cos(imu_yaw_));
    }

    std::pair<int, int> twistToRpm(double linear, double angular)
    {
        // Differential drive kinematics
        // Motor1 = left, Motor2 = right
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

    // Single unified timer: read → publish → send
    // Ensures odom is always computed from the freshest motor state
    void controlCallback()
    {
        if (!driver_->isConnected()) {
            return;
        }

        auto current_time = this->now();

        // 1. Read response from previous cycle's command (freshest data)
        if (driver_->readResponse()) {
            last_comm_time_ = current_time;
            comm_ok_ = true;
        }

        // 2. Publish odometry using fresh motor state
        publishCallback();

        // 3. Check cmd_vel timeout and send next command
        double time_since_cmd = (current_time - last_cmd_time_).seconds();
        if (time_since_cmd > cmd_vel_timeout_) {
            target_linear_ = 0.0;
            target_angular_ = 0.0;
        }
        auto [rpm_left, rpm_right] = twistToRpm(target_linear_, target_angular_);
        if (driver_->sendRpm(rpm_left, rpm_right)) {
            last_comm_time_ = current_time;
            comm_ok_ = true;
        }
    }

    double encoderToMeters(int32_t encoder_ticks)
    {
        // encoder_resolution = counts per motor shaft revolution (measured ~65,536)
        // output shaft revolutions = motor shaft revolutions / gear_ratio
        //                          = (ticks / encoder_resolution) / gear_ratio
        double motor_shaft_revs = static_cast<double>(encoder_ticks) /
                                  static_cast<double>(encoder_resolution_);
        double wheel_revolutions = motor_shaft_revs / static_cast<double>(gear_ratio_);
        double distance = wheel_revolutions * 2.0 * M_PI * wheel_radius_;
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

        double delta_s, delta_theta, left_delta, right_delta;

        if (use_encoder_odom_) {
            // Encoder-based odometry using hall sensor position
            if (!encoder_initialized_) {
                last_left_position_ = left_state.position;
                last_right_position_ = right_state.position;
                encoder_initialized_ = true;
                return;
            }

            int32_t left_delta_ticks = left_state.position - last_left_position_;
            int32_t right_delta_ticks = right_state.position - last_right_position_;
            last_left_position_ = left_state.position;
            last_right_position_ = right_state.position;

            // Motor1=left, Motor2=right (Motor2 position is negated)
            left_delta = encoderToMeters(left_delta_ticks);     // Motor1 = left
            right_delta = -encoderToMeters(right_delta_ticks);  // Motor2 = right (negated)

            delta_s = (left_delta + right_delta) / 2.0;
            delta_theta = (left_delta - right_delta) / wheel_separation_;
        } else {
            // Command-based odometry
            delta_s = target_linear_ * dt;
            delta_theta = target_angular_ * dt;
            left_delta = (target_linear_ - target_angular_ * wheel_separation_ / 2.0) * dt;
            right_delta = (target_linear_ + target_angular_ * wheel_separation_ / 2.0) * dt;
        }

        // Update pose
        if (use_imu_ && imu_initialized_) {
            // TurtleBot3 style: use IMU yaw directly for heading
            // Position still computed from wheel encoders
            x_ += delta_s * std::cos(imu_yaw_ + delta_theta / 2.0);
            y_ += delta_s * std::sin(imu_yaw_ + delta_theta / 2.0);
            theta_ = imu_yaw_;
        } else {
            // Pure wheel odometry (original)
            x_ += delta_s * std::cos(theta_ + delta_theta / 2.0);
            y_ += delta_s * std::sin(theta_ + delta_theta / 2.0);
            theta_ += delta_theta;
            // Normalize theta to [-pi, pi]
            theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
        }

        // Calculate velocities
        double linear_vel = (dt > 0) ? delta_s / dt : 0.0;
        double angular_vel = (dt > 0) ? delta_theta / dt : 0.0;

        // Publish odometry
        publishOdometry(current_time, linear_vel, angular_vel);

        // Publish joint states for wheel TF
        left_wheel_pos_ += left_delta / wheel_radius_;
        right_wheel_pos_ += right_delta / wheel_radius_;
        publishJointStates(current_time);

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

        // Covariance — 값이 클수록 EKF가 덜 신뢰
        // Pose covariance (EKF에서 pose는 안 쓰지만 시각화/다른 노드 참조용)
        odom.pose.covariance[0] = 0.1;    // x
        odom.pose.covariance[7] = 0.1;    // y
        odom.pose.covariance[14] = 1e6;   // z (사용 안 함, 큰 값)
        odom.pose.covariance[21] = 1e6;   // roll
        odom.pose.covariance[28] = 1e6;   // pitch
        odom.pose.covariance[35] = 0.2;   // yaw

        // Twist covariance (EKF가 실제 사용하는 값)
        odom.twist.covariance[0] = 0.05;   // vx
        odom.twist.covariance[7] = 1e6;    // vy (diff-drive: 횡방향 이동 없음)
        odom.twist.covariance[14] = 1e6;   // vz
        odom.twist.covariance[21] = 1e6;   // vroll
        odom.twist.covariance[28] = 1e6;   // vpitch
        odom.twist.covariance[35] = 0.1;   // vyaw

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

    void publishJointStates(const rclcpp::Time& stamp)
    {
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = stamp;
        js.name = {"wheel_left_joint", "wheel_right_joint"};
        js.position = {left_wheel_pos_, right_wheel_pos_};
        js.velocity = {};
        js.effort = {};
        joint_state_pub_->publish(js);
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
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_current_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_position_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_alarm_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
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
    double cmd_vel_timeout_;
    double watchdog_timeout_;

    // Parameters - Frames
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool publish_tf_;
    bool use_encoder_odom_;
    bool use_imu_;

    // Odometry state
    double x_;
    double y_;
    double theta_;
    double left_wheel_pos_{0.0};
    double right_wheel_pos_{0.0};
    int32_t last_left_position_;
    int32_t last_right_position_;
    bool encoder_initialized_{false};
    rclcpp::Time last_time_;

    // IMU state
    double imu_yaw_;
    double imu_yaw_offset_;
    bool imu_initialized_;

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
