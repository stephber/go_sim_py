// Copyright VIF

#include <autoware_interface/autoware_interface.hpp>
#include <autoware_interface/conversions.hpp>
#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <vifware_vehicle_interface_common/utils.hpp>

using namespace vifware_vehicle_interfaces;

using std::placeholders::_1;
using std::placeholders::_2;

AutowareInterface::AutowareInterface(const std::shared_ptr<rclcpp::Node> &node)
    : node_(node)
{
    // From Autoware
    control_cmd_sub_ = node_->create_subscription<AutowareControlCmd>("/control/command/control_cmd", 1,
        [this](const AutowareControlCmd::ConstSharedPtr &msg) {
            data_from_autoware_.ctrl_cmd_accel_ = msg->longitudinal.acceleration;
            data_from_autoware_.ctrl_cmd_speed_ = msg->longitudinal.velocity;
            data_from_autoware_.ctrl_cmd_steering_tire_angle_ = msg->lateral.steering_tire_angle;
        });
    gear_cmd_sub_ = node_->create_subscription<GearCmd>("/control/command/gear_cmd", 1,
        [this](const GearCmd::ConstSharedPtr &msg) {
            data_from_autoware_.gear_status_ = autowareGearCommandToGearStatus(msg->command);
        });
    turn_indicators_cmd_sub_ = node_->create_subscription<TurnIndicatorsCmd>("/control/command/turn_indicators_cmd", 1,
        [this](const TurnIndicatorsCmd::ConstSharedPtr &msg) {
            if (data_from_autoware_.turn_signal_ != TurnSignal::HAZARD) // consider turning indicator signal only if hazard signal is inactive
            {
                data_from_autoware_.turn_signal_ = autowareTurnIndicatorsCommandToTurnSignal(msg->command);
            }
        });
    hazard_lights_cmd_sub_ = node_->create_subscription<HazardLightsCmd>("/control/command/hazard_lights_cmd", 1,
        [this](const HazardLightsCmd::ConstSharedPtr &msg) {
            if (msg->command == autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE) // always activate hazard signal 
            {
                data_from_autoware_.turn_signal_ = TurnSignal::HAZARD;
            }
            else if (data_from_autoware_.turn_signal_ == TurnSignal::HAZARD) // deactivate hazard signal only if it was active
            {
                data_from_autoware_.turn_signal_ = TurnSignal::NONE;
            }
        });
    engage_cmd_sub_ = node_->create_subscription<EngageMsg>("/vehicle/engage", 1,
        [this](const EngageMsg::ConstSharedPtr &msg) {
            data_from_autoware_.engage_ = msg->engage;
        });
    emergency_sub_ = node_->create_subscription<EmergencyMsg>("/control/command/emergency_cmd", 1,
        [this](const EmergencyMsg::ConstSharedPtr &msg) {
            data_from_autoware_.emergency_ = msg->emergency;
        });

    door_command_srv_ = node_->create_service<SetDoorCommand>("/vehicle/doors/command",
                                                              std::bind(&AutowareInterface::cbDoorCommand, this, _1, _2));
    door_layout_srv_  = node_->create_service<GetDoorLayout>("/vehicle/doors/layout",
                                                             std::bind(&AutowareInterface::cbDoorLayout, this, _1, _2));
    ramp_command_srv_ = node_->create_service<SetRampCommand>("/vehicle/ramps/command",
                                                              std::bind(&AutowareInterface::cbRampCommand, this, _1, _2));

    // To Autoware
    control_mode_pub_ = node_->create_publisher<ControlModeRpt>("/vehicle/status/control_mode", rclcpp::QoS{1});
    velocity_rpt_pub_ = node_->create_publisher<VelocityRpt>("/vehicle/status/velocity_status", rclcpp::QoS{1});
    steering_rpt_pub_ = node_->create_publisher<SteeringRpt>("/vehicle/status/steering_status", rclcpp::QoS{1});
    gear_rpt_pub_ = node_->create_publisher<GearRpt>("/vehicle/status/gear_status", rclcpp::QoS{1});
    turn_indicators_rpt_pub_ = node_->create_publisher<TurnIndicatorsRpt>("/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
    hazard_lights_rpt_pub_ = node_->create_publisher<HazardLightsRpt>("/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
    actuation_status_pub_ = node_->create_publisher<ActuationStatus>("/vehicle/status/actuation_status", rclcpp::QoS{1});
    steering_wheel_status_pub_ = node_->create_publisher<SteeringWheelStatus>("/vehicle/status/steering_wheel_status", rclcpp::QoS{1});
    door_status_pub_ = node_->create_publisher<DoorStatusArray>("/vehicle/doors/status", rclcpp::QoS{1}.transient_local());
    ramp_status_pub_ = node_->create_publisher<RampStatusArray>("/vehicle/ramps/status", rclcpp::QoS{1}.transient_local());

    // parameter
    base_frame_id_ = node_->declare_parameter(VIF_PARAM("base_frame_id"), "base_link");
    simulate_turn_indicator_state_ = node_->declare_parameter(VIF_PARAM("simulate_turn_indicator_state"), false);
}

void AutowareInterface::cbDoorCommand(const std::shared_ptr<SetDoorCommand::Request> request,
                  std::shared_ptr<SetDoorCommand::Response> response)
{
    if (set_door_command_cb_)
    {
        set_door_command_cb_(request, response);
    }
    else
    {
        response->status.success = false;
        response->status.code = ResponseStatus::UNKNOWN;
    }
}

void AutowareInterface::cbDoorLayout(const std::shared_ptr<GetDoorLayout::Request> /*request*/,
                                     std::shared_ptr<GetDoorLayout::Response> response)
{
    if (get_door_layout_cb_)
    {
        get_door_layout_cb_(response);
    }
    else
    {
        response->status.success = false;
        response->status.code = ResponseStatus::UNKNOWN;
    }
}

void AutowareInterface::cbRampCommand(const std::shared_ptr<SetRampCommand::Request> request,
                                      std::shared_ptr<SetRampCommand::Response> response)
{
    if (set_ramp_command_cb_)
    {
        set_ramp_command_cb_(request, response);
    }
    else
    {
        response->status.success = false;
        response->status.code = ResponseStatus::UNKNOWN;
    }
}

void AutowareInterface::publish(const ToAutoware &data)
{
    ControlModeRpt control_mode_msg;
    control_mode_msg.stamp = node_->get_clock()->now();
    control_mode_msg.mode = toAutowareControlMode(data.ctrl_mode_);
    control_mode_pub_->publish(control_mode_msg);

    VelocityRpt twist;
    twist.header.stamp = node_->get_clock()->now();
    twist.header.frame_id = base_frame_id_;
    twist.longitudinal_velocity = data.vel_lin_; // [m/s]
    twist.heading_rate = data.vel_ang_; // [rad/s]
    velocity_rpt_pub_->publish(twist);

    SteeringRpt steer_msg;
    steer_msg.stamp = node_->get_clock()->now();
    steer_msg.steering_tire_angle = data.steering_tire_angle_; //rad
    steering_rpt_pub_->publish(steer_msg);

    GearRpt gear_msg;
    gear_msg.stamp = node_->get_clock()->now();
    gear_msg.report = toAutowareGearReport(data.gear_status_);
    gear_rpt_pub_->publish(gear_msg);

    TurnIndicatorsRpt turn_msg;
    turn_msg.stamp = node_->get_clock()->now();

    if (simulate_turn_indicator_state_) {
        turn_msg.report = toAutowareTurnIndicatorsReport(data_from_autoware_.turn_signal_);
    } else {
        turn_msg.report = toAutowareTurnIndicatorsReport(data.turn_signal_);
    }
    turn_indicators_rpt_pub_->publish(turn_msg);

    HazardLightsRpt hazard_msg;
    hazard_msg.stamp =  node_->get_clock()->now();
    if (simulate_turn_indicator_state_) {
        hazard_msg.report = (data_from_autoware_.turn_signal_ == TurnSignal::HAZARD)
            ? toAutowareHazardLightsReport(TurnSignal::HAZARD)
            : toAutowareHazardLightsReport(TurnSignal::NONE);
    } else {
        hazard_msg.report = toAutowareHazardLightsReport(data.turn_signal_);
    }
    hazard_lights_rpt_pub_->publish(hazard_msg);

    ActuationStatus actuation_status;
    actuation_status.header.stamp = node_->get_clock()->now();
    actuation_status.status.accel_status = data.actuation_status_accel_;
    actuation_status.status.brake_status = data.actuation_status_brake_;
    actuation_status.status.steer_status = data.actuation_status_steer_;
    actuation_status_pub_->publish(actuation_status);

    SteeringWheelStatus steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = node_->get_clock()->now();
    steering_wheel_status_msg.data = data.steering_wheel_angle_;
    steering_wheel_status_pub_->publish(steering_wheel_status_msg);

    DoorStatusArray door_status_array_msg;
    door_status_array_msg.stamp = node_->get_clock()->now();
    door_status_array_msg.doors = data.doors_status_;
    door_status_pub_->publish(door_status_array_msg);

    RampStatusArray ramp_status_array_msg;
    ramp_status_array_msg.stamp = node_->get_clock()->now();
    ramp_status_array_msg.ramps = data.ramps_status_;
    ramp_status_pub_->publish(ramp_status_array_msg);
}
