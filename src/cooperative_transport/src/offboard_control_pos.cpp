/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example for 2 Drones
 * @file offboard_control.cpp
 */

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
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

        // Offboard Control Mode Publishers (Required to tell PX4 we are sending Position Setpoints)
        offboard_control_mode_pub_1_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        offboard_control_mode_pub_2_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);

        // Trajectory Setpoint Publishers
        trajectory_setpoint_pub_1_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        trajectory_setpoint_pub_2_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);

        timer_ticks_ = 0;

        auto timer_callback = [this]() -> void {
            // Offboard mode requires streaming setpoints at >2Hz BEFORE engaging offboard mode.
            // We run this timer at 20Hz (50ms).
            
            // 1. Send heartbeat signals (OffboardControlMode)
            publish_offboard_control_mode();

            // 2. State Machine for the Flight Plan
            if (timer_ticks_ < 200) { 
                // Phase 1: 0 to 10 seconds - Takeoff and Hover
                // Drone 1: ENU [-0.5, 2.0, 2.5] -> NED [2.0, -0.5, -2.5]
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, 0.0, 0.0, -2.5);
                // Drone 2: ENU [0.5, 2.0, 2.5] -> NED [2.0, 0.5, -2.5]
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, 0.0, 0.0, -2.5);

                // Arm and switch to Offboard mode after 1 second of streaming setpoints
                if (timer_ticks_ == 20) {
                    this->arm();
                    this->set_offboard_mode();
                    RCLCPP_INFO(this->get_logger(), "Engaging OFFBOARD mode and taking off...");
                }
            } 
            else if (timer_ticks_ < 400) {
                // Phase 2: 10 to 20 seconds - Move Forward 1m (+2m in ENU X)
                if (timer_ticks_ == 200) {
                    RCLCPP_INFO(this->get_logger(), "Moving Forward 2m...");
                }
                // Drone 1: ENU [1.5, 2.0, 2.5] -> NED [0.0, 2.0, -2.5]
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, 0.0, 2.0, -2.5);
                // Drone 2: ENU [2.5, 2.0, 2.5] -> NED [0.0, 2.0, -2.5]
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, 0.0, 2.0, -2.5);
            } 
            else {
                // Phase 3: 20 seconds onwards - Move Left 3m (+3m in ENU Y)
                if (timer_ticks_ == 400) {
                    RCLCPP_INFO(this->get_logger(), "Moving Left 3m...");
                }
                // Drone 1: ENU [1.5, 5.0, 2.5] -> NED [3.0, 2.0, -2.5]
                publish_trajectory_setpoint(trajectory_setpoint_pub_1_, 3.0, 2.0, -2.5);
                // Drone 2: ENU [2.5, 5.0, 2.5] -> NED [3.0, 2.0, -2.5]
                publish_trajectory_setpoint(trajectory_setpoint_pub_2_, 3.0, 2.0, -2.5);
            }

            timer_ticks_++;
        };
        
        // 50ms timer = 20Hz loop rate
        timer_ = this->create_wall_timer(50ms, timer_callback);
    }

    void arm();
    void set_offboard_mode();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t timer_ticks_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_2_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_1_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_2_;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_1_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_2_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(const rclcpp::Publisher<TrajectorySetpoint>::SharedPtr & publisher, float x_ned, float y_ned, float z_ned);
    
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
    // MAV_CMD_DO_SET_MODE: custom mode enabled, OFFBOARD mode
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
    float x_ned, float y_ned, float z_ned)
{
    TrajectorySetpoint msg{};
    
    // Explicitly set all velocity, acceleration, and jerk fields to NaN as required
    msg.velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    msg.acceleration = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    msg.jerk = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    
    msg.position = {x_ned, y_ned, z_ned};
    
    // Keep drones facing same as initial heading (yaw = 90 degrees correespond to yaw 0 in gazebo world frame)
    msg.yaw = 1.57f; 
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
    std::cout << "Starting multi-drone offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}