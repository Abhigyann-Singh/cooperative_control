/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example for 2 Drones with Dynamic Rod Input
 * @file offboard_control.cpp
 */

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <limits>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
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

        // Subscriber for Dynamic Target [x, y, z, yaw, pitch]
        target_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rod_target", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 5) {
                    target_x_ = msg->data[0];
                    target_y_ = msg->data[1];
                    target_z_ = msg->data[2];
                    target_yaw_ = msg->data[3];
                    target_pitch_ = msg->data[4];
                    RCLCPP_INFO(this->get_logger(), "New Rod Target: X:%.2f Y:%.2f Z:%.2f Yaw:%.2f Pitch:%.2f",
                                target_x_, target_y_, target_z_, target_yaw_, target_pitch_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid target array size. Expected 5 elements.");
                }
            });

        timer_ticks_ = 0;

        auto timer_callback = [this]() -> void {
            // 1. Send heartbeat signals
            publish_offboard_control_mode();

            // 2. Calculate Geometry for Rod Ends
            // Rod length = 1.0m, so radius = 0.5m. 
            // Assuming drone 1 is at -X and drone 2 is at +X when yaw=0, pitch=0
            float dx = 0.5f * cos(-target_pitch_) * cos(target_yaw_);
            float dy = 0.5f * cos(-target_pitch_) * sin(target_yaw_);
            float dz = 0.5f * sin(-target_pitch_);

            // Absolute ENU positions for both drones
            float d1_enu_x = target_x_ - dx;
            float d1_enu_y = target_y_ - dy;
            float d1_enu_z = target_z_ - dz;

            float d2_enu_x = target_x_ + dx;
            float d2_enu_y = target_y_ + dy;
            float d2_enu_z = target_z_ + dz;

            // 3. Convert Absolute ENU to Local NED Setpoints
            // ENU -> NED conversion: X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
            // Then subtract the initial absolute NED power-on positions of each drone.
            
            // Drone 1 Initial NED: [2.0, -0.5, 0.0]
            float d1_sp_x = d1_enu_y - 2.0f;
            float d1_sp_y = d1_enu_x - (-0.5f);
            float d1_sp_z = -d1_enu_z;

            // Drone 2 Initial NED: [2.0, 0.5, 0.0]
            float d2_sp_x = d2_enu_y - 2.0f;
            float d2_sp_y = d2_enu_x - 0.5f;
            float d2_sp_z = -d2_enu_z;

            // 4. Handle Yaw Default Offset (0 rad input -> 1.57 rad in PX4)
            float adjusted_yaw = -target_yaw_ + 1.57f;

            // 5. State Machine: Stream setpoints then arm
            if (timer_ticks_ < 20) {
                // Stream setpoints before arming
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, d1_sp_x, d1_sp_y, d1_sp_z, adjusted_yaw);
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, d2_sp_x, d2_sp_y, d2_sp_z, adjusted_yaw);
            } 
            else if (timer_ticks_ == 20) {
                this->arm();
                this->set_offboard_mode();
                RCLCPP_INFO(this->get_logger(), "Engaging OFFBOARD mode and taking off...");
            } 
            else {
                // Continually stream dynamic setpoints
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, d1_sp_x, d1_sp_y, d1_sp_z, adjusted_yaw);
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, d2_sp_x, d2_sp_y, d2_sp_z, adjusted_yaw);
            }

            timer_ticks_++;
        };
        
        // 20ms timer = 50Hz control loop
        timer_ = this->create_wall_timer(20ms, timer_callback);
    }

    void arm();
    void set_offboard_mode();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t timer_ticks_;

    // Target state variables (Default: Hover at [0, 2.0, 2.5] ENU)
    float target_x_ = 0.0f;
    float target_y_ = 2.0f;
    float target_z_ = 2.5f;
    float target_yaw_ = 0.0f;
    float target_pitch_ = 0.0f;

    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_2_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_1_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_2_;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_1_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_2_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_subscriber_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(const rclcpp::Publisher<TrajectorySetpoint>::SharedPtr & publisher, float x_ned, float y_ned, float z_ned, float yaw_rad);
    
    void publish_vehicle_command(
        const rclcpp::Publisher<VehicleCommand>::SharedPtr & publisher,
        uint8_t target_system,
        uint16_t command,
        float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0,
        float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
};

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
    std::cout << "Starting multi-drone offboard rod control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}