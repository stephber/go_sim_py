// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <array>


#include <rclcpp/rclcpp.hpp>
#include "omnidrive_control/odometry.hpp"
#include "omnidrive_control/wheel.hpp"
#include "omnidrive_control_parameters.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "vifware_vehicle_interface_common/error_handling.hpp"

namespace vifware_vehicle_interfaces
{

    constexpr auto NUM_OMNI_WHEELS = 4;
    enum class OmniWheel : uint8_t {
        front_right = 0,
        back_right  = 1,
        front_left  = 2,
        back_left   = 3
    };

    struct OmniWheelState {
        double rot;
        double vel;
    };
    using OmniWheelStates = std::array<OmniWheelState, NUM_OMNI_WHEELS>;


    class OmniDriveControl
    {
        using Wheels = std::vector<std::shared_ptr<Wheel>>;

    public:

        explicit OmniDriveControl(const std::shared_ptr<rclcpp::Node> &node);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

        ReturnT update(const rclcpp::Time & time, const OmniWheelStates& states);

        ReturnT update(const rclcpp::Time & time,
                       const OmniWheelStates& states,
                       const std::shared_ptr<geometry_msgs::msg::Twist> cmd);

        OmniWheelStates getCtrlCmd() const;
        geometry_msgs::msg::Twist getVelocityState() const;

protected:
  // Parameters from ROS for omnidrive_control
        std::shared_ptr<omnidrive_control::ParamListener> param_listener_;
        omnidrive_control::Params params_;

    private:
        void initialize();

        ReturnT loadParams();
        ReturnT initComm();
        ReturnT initOdometry();

        rclcpp::Node::SharedPtr node_;

        Odometry odometry_;

        // Timeout to consider cmd_vel commands old
        std::chrono::milliseconds cmd_vel_timeout_{500};

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

        std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
            nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
        realtime_odometry_transform_publisher_ = nullptr;

        bool subscriber_is_active_ = false;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        velocity_command_unstamped_subscriber_ = nullptr;

        realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> received_velocity_msg_ptr_{nullptr};

        std::queue<geometry_msgs::msg::TwistStamped> previous_commands_;  // last two commands

        rclcpp::Time previous_update_timestamp_{0};

        // publish rate limiter
        double publish_rate_ = 50.0;
        rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
        rclcpp::Time previous_publish_timestamp_{0};

        bool is_halted = false;
        bool use_stamped_vel_ = true;

        ReturnT reset();
        ReturnT halt();

        Wheels wheels_;
    };

}  // namespace vifware_vehicle_interfaces
