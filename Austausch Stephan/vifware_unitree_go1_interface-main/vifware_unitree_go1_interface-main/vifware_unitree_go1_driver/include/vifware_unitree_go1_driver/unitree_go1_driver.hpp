// Copyright VIF

#pragma once

#include<memory>

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include <autoware_interface/autoware_ad_api_client.hpp>
#include <autoware_interface/autoware_interface.hpp>

// #include <vifware_vehicle_interface_msgs/msg/autoware_interface_control_command.hpp>
// #include <vifware_vehicle_interface_msgs/msg/autoware_interface_misc_command.hpp>
// #include <vifware_vehicle_interface_msgs/msg/autoware_interface_report.hpp>

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

#include "vifware_unitree_go1_driver/enums.hpp"


namespace vifware_unitree_go1_interface {

    // "node-like" cf.
    // https://github.com/ros2/demos/blob/rolling/composition/src/node_like_listener_component.cpp
    class UnitreeGo1Driver
    {
    public:
        explicit UnitreeGo1Driver(const rclcpp::NodeOptions & options);

        // implement component
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
        get_node_base_interface() const;

        // //internal
        // using AutowareIfControlCommand = vifware_vehicle_interface_msgs::msg::AutowareInterfaceControlCommand;
        // using AutowareIfMiscCommand = vifware_vehicle_interface_msgs::msg::AutowareInterfaceMiscCommand;
        // using AutowareIfReport = vifware_vehicle_interface_msgs::msg::AutowareInterfaceReport;

    private:
        void initComm();
        void initTimer();
        void timerCallback();
        void publishToAutowareInterface();
        void initParams();

        void reset_state();
        void reset_cmd_vel();

        // From UnitreeGo1 UDP Bridge
        rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_sub_;

        // To UnitreeGo1 UDP Bridge
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_pub_;

        // internal communication
        // rclcpp::Subscription<AutowareIfControlCommand>::SharedPtr autoware_if_control_command_sub_;
        // rclcpp::Subscription<AutowareIfMiscCommand>::SharedPtr autoware_if_misc_command_sub_;
        // rclcpp::Publisher<AutowareIfReport>::SharedPtr autoware_if_report_pub_;

        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr autoware_twist_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr autoware_imu_pub_;

        // implement component
        rclcpp::Node::SharedPtr node_;

        // access to other components
        std::shared_ptr<vifware_vehicle_interfaces::AutowareAdApiClient> ad_client_;
        std::shared_ptr<vifware_vehicle_interfaces::AutowareInterface> aw_if_;

        //parameter
        double loop_rate_hz_;
        double wheelbase_;
        rclcpp::TimerBase::SharedPtr loop_timer_;

        // cache inputs / outputs
        // AutowareIfControlCommand::ConstSharedPtr aw_ctrl_cmd_;
        // AutowareIfMiscCommand::ConstSharedPtr aw_misc_cmd_;
        vifware_vehicle_interfaces::ToAutoware data_to_autoware_;
        ros2_unitree_legged_msgs::msg::HighState::ConstSharedPtr high_state_;
        ros2_unitree_legged_msgs::msg::HighCmd high_cmd_;
        geometry_msgs::msg::Twist cmd_vel_;


        // state variables <--- todo move to FSM
        bool reset_state_ = false;
        uint8_t reset_counter_ = 0;
        Go1Gait walking_gait_;

        // services
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_state_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stand_up_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_recover_stand_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_lay_down_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_damping_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_jump_yaw_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_beg_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_dance1_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_dance2_;
        // rclcpp::Service<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr srv_set_body_rpy_;
        // rclcpp::Service<unitree_nav_interfaces::srv::SetGait>::SharedPtr srv_set_gait_;

    };

} // end namespace
