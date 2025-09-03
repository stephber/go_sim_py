// Copyright VIF

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/door_command.hpp>
#include <autoware_adapi_v1_msgs/msg/door_layout.hpp>
#include <autoware_adapi_v1_msgs/msg/door_status_array.hpp>
#include <autoware_adapi_v1_msgs/srv/get_door_layout.hpp>
#include <autoware_adapi_v1_msgs/srv/set_door_command.hpp>
#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <vifware_msgs/msg/ramp_command.hpp>
#include <vifware_msgs/msg/ramp_status_array.hpp>
#include <vifware_msgs/srv/set_ramp_command.hpp>

namespace vifware_vehicle_interfaces
{
    enum class GearStatus : uint8_t
    {
        NONE = 0,
        PARK = 1,
        REVERSE = 2,
        NEUTRAL = 3,
        DRIVE = 4,
        LOW = 5,
        SPORT = 6
    };

    enum class TurnSignal : uint8_t
    {
        NONE = 0,
        LEFT = 1,
        RIGHT = 2,
        HAZARD = 3
    };

    enum class ControlMode : uint8_t
    {
        NONE = 0,
        AUTONOMOUS = 1,
        MANUAL = 2,
        ERROR = 3
    };

    struct FromAutoware
    {
        bool emergency_ = false;
        bool engage_ = false;

        double ctrl_cmd_accel_ = 0;
        double ctrl_cmd_steering_tire_angle_ = 0;
        double ctrl_cmd_speed_ = 0;

        GearStatus gear_status_ = GearStatus::PARK;
        TurnSignal turn_signal_ = TurnSignal::NONE;
    };

    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using DoorCommand = autoware_adapi_v1_msgs::msg::DoorCommand;
    using DoorLayout = autoware_adapi_v1_msgs::msg::DoorLayout;
    using DoorStatus = autoware_adapi_v1_msgs::msg::DoorStatus;
    using SetDoorCommand = autoware_adapi_v1_msgs::srv::SetDoorCommand;
    using GetDoorLayout = autoware_adapi_v1_msgs::srv::GetDoorLayout;
    using DoorStatusArray = autoware_adapi_v1_msgs::msg::DoorStatusArray;
    using RampCommand = vifware_msgs::msg::RampCommand;
    using RampStatus = vifware_msgs::msg::RampStatus;
    using RampStatusArray = vifware_msgs::msg::RampStatusArray;
    using SetRampCommand = vifware_msgs::srv::SetRampCommand;

    struct ToAutoware
    {
        bool actuation_status_accel_ = false;
        bool actuation_status_brake_ = false;
        bool actuation_status_steer_ = false;

        ControlMode ctrl_mode_ = ControlMode::NONE;

        double steering_tire_angle_ = 0;
        double steering_wheel_angle_ = 0;
        double vel_ang_ = 0;
        double vel_lin_ = 0;

        GearStatus gear_status_ = GearStatus::PARK;
        TurnSignal turn_signal_ = TurnSignal::NONE;

        std::vector<DoorStatus> doors_status_;
        std::vector<RampStatus> ramps_status_;
    };

    class AutowareInterface
    {
    public:
        static constexpr const char *PARAM_NS = "aw_if.";

        explicit AutowareInterface(const std::shared_ptr<rclcpp::Node> &node);

        // From Autoware
        FromAutoware data_from_autoware_;
        using AutowareControlCmd = autoware_control_msgs::msg::Control;
        using GearCmd = autoware_vehicle_msgs::msg::GearCommand;
        using TurnIndicatorsCmd = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
        using HazardLightsCmd = autoware_vehicle_msgs::msg::HazardLightsCommand;
        using EngageMsg = autoware_vehicle_msgs::msg::Engage;
        using EmergencyMsg = tier4_vehicle_msgs::msg::VehicleEmergencyStamped;

        // To Autoware
        void publish(const ToAutoware &data);
        using ControlModeRpt = autoware_vehicle_msgs::msg::ControlModeReport;
        using VelocityRpt = autoware_vehicle_msgs::msg::VelocityReport;
        using SteeringRpt = autoware_vehicle_msgs::msg::SteeringReport;
        using GearRpt = autoware_vehicle_msgs::msg::GearReport;
        using TurnIndicatorsRpt = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
        using HazardLightsRpt = autoware_vehicle_msgs::msg::HazardLightsReport;
        using ActuationStatus = tier4_vehicle_msgs::msg::ActuationStatusStamped;
        using SteeringWheelStatus = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;

        void setCallbackGetDoorLayout(std::function<void(GetDoorLayout::Response::SharedPtr)> cb) {
            get_door_layout_cb_ = cb;
        }

        void setCallbackSetDoorCommand(std::function<void(const SetDoorCommand::Request::SharedPtr,
                                                          SetDoorCommand::Response::SharedPtr)> cb) {
            set_door_command_cb_ = cb;
        }

        void setCallbackSetRampCommand(std::function<void(const SetRampCommand::Request::SharedPtr,
                                                          SetRampCommand::Response::SharedPtr)> cb) {
            set_ramp_command_cb_ = cb;
        }

    private:
        // From Autoware
        rclcpp::Subscription<AutowareControlCmd>::SharedPtr control_cmd_sub_;
        rclcpp::Subscription<GearCmd>::SharedPtr gear_cmd_sub_;
        rclcpp::Subscription<TurnIndicatorsCmd>::SharedPtr turn_indicators_cmd_sub_;
        rclcpp::Subscription<HazardLightsCmd>::SharedPtr hazard_lights_cmd_sub_;
        rclcpp::Subscription<EngageMsg>::SharedPtr engage_cmd_sub_;
        rclcpp::Subscription<EmergencyMsg>::SharedPtr emergency_sub_;

        // To Autoware
        rclcpp::Publisher<ControlModeRpt>::SharedPtr control_mode_pub_;
        rclcpp::Publisher<VelocityRpt>::SharedPtr velocity_rpt_pub_;
        rclcpp::Publisher<SteeringRpt>::SharedPtr steering_rpt_pub_;
        rclcpp::Publisher<GearRpt>::SharedPtr gear_rpt_pub_;
        rclcpp::Publisher<TurnIndicatorsRpt>::SharedPtr turn_indicators_rpt_pub_;
        rclcpp::Publisher<HazardLightsRpt>::SharedPtr hazard_lights_rpt_pub_;
        rclcpp::Publisher<ActuationStatus>::SharedPtr actuation_status_pub_;
        rclcpp::Publisher<SteeringWheelStatus>::SharedPtr steering_wheel_status_pub_;
        rclcpp::Publisher<DoorStatusArray>::SharedPtr door_status_pub_;
        rclcpp::Publisher<RampStatusArray>::SharedPtr ramp_status_pub_;

        // Services
        rclcpp::Service<SetDoorCommand>::SharedPtr door_command_srv_;
        rclcpp::Service<GetDoorLayout>::SharedPtr door_layout_srv_;
        rclcpp::Service<SetRampCommand>::SharedPtr ramp_command_srv_;

        // implement component
        rclcpp::Node::SharedPtr node_;

        // parameter
        std::string base_frame_id_;

        // callbacks
        std::function<void(GetDoorLayout::Response::SharedPtr)> get_door_layout_cb_;
        std::function<void(const SetDoorCommand::Request::SharedPtr,
                           SetDoorCommand::Response::SharedPtr)> set_door_command_cb_;
        std::function<void(const SetRampCommand::Request::SharedPtr,
                           SetRampCommand::Response::SharedPtr)> set_ramp_command_cb_;

        void cbDoorCommand(const std::shared_ptr<SetDoorCommand::Request> request,
                          std::shared_ptr<SetDoorCommand::Response> response);

        void cbDoorLayout(const std::shared_ptr<GetDoorLayout::Request> request,
                          std::shared_ptr<GetDoorLayout::Response> response);
        void cbRampCommand(const std::shared_ptr<SetRampCommand::Request> request,
                           std::shared_ptr<SetRampCommand::Response> response);
        bool simulate_turn_indicator_state_;
    };

} // namespace vifware_vehicle_interfaces
