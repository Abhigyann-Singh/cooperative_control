/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @brief Offboard control for 2 Drones with Rod PID Feedback
 * @file offboard_control.cpp
 */

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <limits>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// Simple PID Controller Class
class PIDController {
public:
    float kp, ki, kd;
    float integral_limit;
    
    PIDController(float p, float i, float d, float max_i = 1.0f) 
        : kp(p), ki(i), kd(d), integral_limit(max_i), integral_(0.0f), prev_error_(0.0f) {}

    float update(float setpoint, float measured_value, float dt) {
        float error = setpoint - measured_value;
        
        // Proportional
        float p_out = kp * error;
        
        // Integral with anti-windup
        integral_ += error * dt;
        if (integral_ > integral_limit) integral_ = integral_limit;
        if (integral_ < -integral_limit) integral_ = -integral_limit;
        float i_out = ki * integral_;
        
        // Derivative
        float derivative = (error - prev_error_) / dt;
        float d_out = kd * derivative;
        
        prev_error_ = error;
        
        return p_out + i_out + d_out;
    }

private:
    float integral_;
    float prev_error_;
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control"),
        // Initialize PIDs: Kp, Ki, Kd, Max_Integral (These WILL need tuning!)
        pid_x(0.8f, 0.0f, 0.05f),
        pid_y(0.8f, 0.0f, 0.05f),
        pid_z(1.0f, 0.0f, 0.2f),
        pid_yaw(0.5f, 0.0f, 0.05f),
        pid_pitch(0.5f, 0.0f, 0.05f)
    {
        // Command Publishers
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_command_publisher_2_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);

        // Offboard Control Mode Publishers
        offboard_control_mode_pub_1_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        offboard_control_mode_pub_2_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);

        // Trajectory Setpoint Publishers
        trajectory_setpoint_pub_1_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        trajectory_setpoint_pub_2_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);

        // Subscriber for Desired Target [x, y, z, yaw, pitch]
        target_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rod_target", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 5) {
                    target_x_ = msg->data[0];
                    target_y_ = msg->data[1];
                    target_z_ = msg->data[2];
                    target_yaw_ = msg->data[3];
                    target_pitch_ = msg->data[4];
                }
            });

        // Subscriber for Actual Rod Pose (Feedback)
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/model/rod_payload/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                actual_x_ = msg->pose.position.x;
                actual_y_ = msg->pose.position.y;
                actual_z_ = msg->pose.position.z + 0.5f;

                // Quaternion to Euler (Pitch and Yaw)
                double qx = msg->pose.orientation.x;
                double qy = msg->pose.orientation.y;
                double qz = msg->pose.orientation.z;
                double qw = msg->pose.orientation.w;

                // Pitch (y-axis rotation)
                double sinp = 2.0 * (qw * qy - qz * qx);
                if (std::abs(sinp) >= 1)
                    actual_pitch_ = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
                else
                    actual_pitch_ = std::asin(sinp);

                // Yaw (z-axis rotation)
                double siny_cosp = 2.0 * (qw * qz + qx * qy);
                double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                actual_yaw_ = std::atan2(siny_cosp, cosy_cosp);
            });

        timer_ticks_ = 0;

        auto timer_callback = [this]() -> void {
            publish_offboard_control_mode();
            // 20ms loop rate
            float dt = 0.02f;

            // 1. Calculate PID Corrections
            // The PID output is added to the feedforward target to create an "adjusted" target
            float adj_x = target_x_ + pid_x.update(target_x_, actual_x_, dt);
            float adj_y = target_y_ + pid_y.update(target_y_, actual_y_, dt);
            float adj_z = target_z_ + pid_z.update(target_z_, actual_z_, dt);
            float adj_yaw = target_yaw_ + pid_yaw.update(target_yaw_, actual_yaw_, dt);
            float adj_pitch = target_pitch_ + pid_pitch.update(target_pitch_, actual_pitch_, dt);

            // 2. Calculate Geometry for Rod Ends using ADJUSTED targets
            float dx = 0.5f * cos(-adj_pitch) * cos(adj_yaw);
            float dy = 0.5f * cos(-adj_pitch) * sin(adj_yaw);
            float dz = 0.5f * sin(-adj_pitch);

            float d1_enu_x = adj_x - dx;
            float d1_enu_y = adj_y - dy;
            float d1_enu_z = adj_z - dz;

            float d2_enu_x = adj_x + dx;
            float d2_enu_y = adj_y + dy;
            float d2_enu_z = adj_z + dz;

            // 3. Convert Absolute ENU to Local NED Setpoints
            float d1_sp_x = d1_enu_y - 2.0f;
            float d1_sp_y = d1_enu_x - (-0.5f);
            float d1_sp_z = -d1_enu_z;

            float d2_sp_x = d2_enu_y - 2.0f;
            float d2_sp_y = d2_enu_x - 0.5f;
            float d2_sp_z = -d2_enu_z;

            // 4. Handle Yaw Default Offset (Adjusted yaw + 1.57 for PX4 alignment)
            float final_yaw_sp = -adj_yaw + 1.57f;

            // 5. State Machine
            if (timer_ticks_ < 20) {
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, d1_sp_x, d1_sp_y, d1_sp_z, final_yaw_sp);
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, d2_sp_x, d2_sp_y, d2_sp_z, final_yaw_sp);
            } 
            else if (timer_ticks_ == 20) {
                this->arm();
                this->set_offboard_mode();
                RCLCPP_INFO(this->get_logger(), "Engaging OFFBOARD mode...");
            } 
            else {
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, d1_sp_x, d1_sp_y, d1_sp_z, final_yaw_sp);
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, d2_sp_x, d2_sp_y, d2_sp_z, final_yaw_sp);
            }

            // Print debug info every ~1 second (20 ticks)
            if (timer_ticks_ % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "Err Z: %.2f | Err Yaw: %.2f", (target_z_ - actual_z_), (target_yaw_ - actual_yaw_));
            }

            timer_ticks_++;
        };
        
        timer_ = this->create_wall_timer(20ms, timer_callback);
    }

    void arm();
    void set_offboard_mode();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t timer_ticks_;

    // Target state variables
    float target_x_ = 0.0f; float target_y_ = 2.0f; float target_z_ = 2.5f;
    float target_yaw_ = 0.0f; float target_pitch_ = 0.0f;

    // Actual state variables (updated from Gazebo)
    float actual_x_ = 0.0f; float actual_y_ = 0.0f; float actual_z_ = 0.0f;
    float actual_yaw_ = 0.0f; float actual_pitch_ = 0.0f;

    // PID Controllers
    PIDController pid_x, pid_y, pid_z, pid_yaw, pid_pitch;

    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_2_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_1_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_2_;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_1_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_2_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(const rclcpp::Publisher<TrajectorySetpoint>::SharedPtr & pub, float x, float y, float z, float yaw);
    void publish_vehicle_command(const rclcpp::Publisher<VehicleCommand>::SharedPtr & pub, uint8_t sys, uint16_t cmd, float p1=0, float p2=0, float p3=0, float p4=0, float p5=0, float p6=0, float p7=0);
};

// ... [Keep the arm(), set_offboard_mode(), publish_offboard_control_mode(), publish_trajectory_setpoint(), and publish_vehicle_command() methods exactly the same as the previous response] ...
void OffboardControl::arm()
{
    publish_vehicle_command(vehicle_command_publisher_, 1, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    publish_vehicle_command(vehicle_command_publisher_2_, 2, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent to both vehicles");
}

void OffboardControl::set_offboard_mode()
{
    publish_vehicle_command(vehicle_command_publisher_, 1, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    publish_vehicle_command(vehicle_command_publisher_2_, 2, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    offboard_control_mode_pub_1_->publish(msg);
    offboard_control_mode_pub_2_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint(
    const rclcpp::Publisher<TrajectorySetpoint>::SharedPtr & publisher, 
    float x_ned, float y_ned, float z_ned, float yaw_rad)
{
    TrajectorySetpoint msg{};
    
    msg.velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    msg.acceleration = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    msg.jerk = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    
    msg.position = {x_ned, y_ned, z_ned};
    msg.yaw = yaw_rad; 
    msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publisher->publish(msg);
}

void OffboardControl::publish_vehicle_command(
    const rclcpp::Publisher<VehicleCommand>::SharedPtr & publisher,
    uint8_t target_system,
    uint16_t command,
    float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    VehicleCommand msg{};
    msg.param1 = param1; msg.param2 = param2; msg.param3 = param3; msg.param4 = param4;
    msg.param5 = param5; msg.param6 = param6; msg.param7 = param7;
    msg.command = command;
    msg.target_system = target_system;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publisher->publish(msg);
}
int main(int argc, char *argv[])
{
    std::cout << "Starting closed-loop PID multi-drone node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}