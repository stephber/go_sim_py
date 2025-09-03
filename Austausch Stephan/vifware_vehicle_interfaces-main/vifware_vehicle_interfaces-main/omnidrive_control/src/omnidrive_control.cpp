// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "omnidrive_control/omnidrive_control.hpp"

#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "/vehicle/spider/cmd/in_stamped";
    constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/vehicle/spider/cmd/in";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "/vehicle/spider/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
    constexpr auto NUM_WHEELS = 4;
}  // namespace

using namespace vifware_vehicle_interfaces;
using namespace std::chrono_literals;

OmniDriveControl::OmniDriveControl(const std::shared_ptr<rclcpp::Node> &node)
    : node_(node)
{
    initialize();
}

// implement component
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OmniDriveControl::get_node_base_interface() const
{
    return this->node_->get_node_base_interface();
}

void OmniDriveControl::initialize()
{
    loadParams()
        .or_else([this](auto& e) {
            RCLCPP_ERROR(node_->get_logger(), toString(e));
        });
    initOdometry()
        .or_else([this](auto& e) {
            RCLCPP_ERROR(node_->get_logger(), toString(e));
        });

    initComm()
        .or_else([this](auto& e) {
            RCLCPP_ERROR(node_->get_logger(), toString(e));
        });

    is_halted = false;
    subscriber_is_active_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");

}

ReturnT OmniDriveControl::loadParams()
{
    try {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<omnidrive_control::ParamListener>(node_->get_node_parameters_interface());
        params_ = param_listener_->get_params();
    } catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }

    auto logger = node_->get_logger();
    // update parameters if they have changed
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        RCLCPP_INFO(logger, "Parameters were updated");
    }

    if (params_.wheels_count != NUM_WHEELS) {
        RCLCPP_ERROR(logger, "Param wheels_count needs to be four!");
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }
    if (params_.wheels_name.size() != NUM_WHEELS) {
        RCLCPP_ERROR(logger, "Param wheels_name array size needs to be four!");
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }
    if (params_.wheels_max_angle.size() != NUM_WHEELS) {
        RCLCPP_ERROR(logger, "Param wheels_max_angle array size needs to be four!");
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }
    if (params_.wheels_offset_x.size() != NUM_WHEELS) {
        RCLCPP_ERROR(logger, "Param wheels_offset_x size needs to be four!");
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }
    if (params_.wheels_offset_y.size() != NUM_WHEELS) {
        RCLCPP_ERROR(logger, "Param wheels_offset_y array size needs to be four!");
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }
    if (params_.wheels_radius.size() != NUM_WHEELS) {
        RCLCPP_ERROR(logger, "Param wheels_radius array size needs to be four!");
        return ErrorT(ErrorEnum::PARAMETER_ERROR);
    }
    // if (params_. wheels_invert.size() != NUM_WHEELS) {
    //     RCLCPP_ERROR(logger, "Param wheels_invert array size needs to be four!");
    //     return ErrorT(ErrorEnum::PARAMETER_ERROR);
    // }
    RCLCPP_INFO_STREAM(logger, "Initializing omni drive controller with " << params_.wheels_count << " wheels.");

    return SuccessT{};
}


ReturnT OmniDriveControl::initComm()
{
    // initialize command subscriber
    if (use_stamped_vel_) {
        velocity_command_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>
            (DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
             [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void
             {
                 if (!subscriber_is_active_) {
                     RCLCPP_WARN(node_->get_logger(),
                                 "Can't accept new commands. subscriber is inactive");
                     return;
                 }
                 if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                     RCLCPP_WARN_ONCE(node_->get_logger(),
                                      "Received TwistStamped with zero timestamp, setting it to current "
                                      "time, this message will only be shown once");
                     msg->header.stamp = node_->get_clock()->now();
                 }
                 received_velocity_msg_ptr_.set(std::move(msg));
             });
    } else {
        velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>
            (DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
             [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
             {
                 if (!subscriber_is_active_) {
                     RCLCPP_WARN(node_->get_logger(),
                                 "Can't accept new commands. subscriber is inactive");
                     return;
                 }

                 // Write fake header in the stored stamped command
                 std::shared_ptr<geometry_msgs::msg::TwistStamped> twist_stamped;
                 received_velocity_msg_ptr_.get(twist_stamped);
                 twist_stamped->twist = *msg;
                 twist_stamped->header.stamp = node_->get_clock()->now();
             });
    }

    // initialize odometry publisher and messasge
    odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>
        (DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

    std::string controller_namespace = std::string(node_->get_namespace());

    if (controller_namespace == "/") {
        controller_namespace = "";
    } else {
        controller_namespace = controller_namespace + "/";
    }

    const auto odom_frame_id = controller_namespace + params_.odom_frame_id;
    const auto base_frame_id = controller_namespace + params_.base_frame_id;

    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = controller_namespace + odom_frame_id;
    odometry_message.child_frame_id = controller_namespace + base_frame_id;

    // limit the publication on the topics /odom and /tf
    publish_rate_ = node_->get_parameter("publish_rate").as_double();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    previous_publish_timestamp_ = node_->get_clock()->now();

    // initialize odom values zeros
    odometry_message.twist =
        geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index) {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;
        odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
    }

    // initialize transform publisher and message
    odometry_transform_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>
        (DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
        (odometry_transform_publisher_);

    // keeping track of odom and base_link transforms only
    auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

    previous_update_timestamp_ = node_->get_clock()->now();

    return SuccessT{};
}

ReturnT OmniDriveControl::initOdometry() {

    auto logger = node_->get_logger();
    // get param
    int wheels_count = params_.wheels_count;
    auto wheels_name = params_.wheels_name;
    auto wheels_max_angle = params_.wheels_max_angle;
    auto wheels_offset_x = params_.wheels_offset_x;
    auto wheels_offset_y = params_.wheels_offset_y;
    auto wheels_radius = params_.wheels_radius;
    auto wheels_invert = params_.wheels_invert;

    for (int i = 0; i < wheels_count; ++i) {
        std::string wheel_name = wheels_name[i];
        RCLCPP_INFO_STREAM(logger, "[omnidirive_controller] Initializing Odometry" << wheel_name);

        double max_rot_angle =  std::fabs(wheels_max_angle[i]);
        double offset_x = wheels_offset_x[i];
        double offset_y = wheels_offset_y[i];
        double radius = wheels_radius[i];
        double invert = wheels_invert[i];

        double wheel_rotation_factor = 2 * M_PI * radius;
        auto wheel = std::make_shared<Wheel>(logger);
        if (wheel->configure(wheel_name, max_rot_angle, offset_x, offset_y,
                             wheel_rotation_factor, invert)) {
            wheels_.push_back(wheel);
        } else {
            RCLCPP_ERROR_STREAM(logger, "[omnidirive_controller] Could not initialize " << wheel_name);
            return ErrorT(ErrorEnum::PARAMETER_ERROR);
        }
        odometry_.addWheel(wheel);
    }

    odometry_.init(node_->get_clock()->now());
    odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
    use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

    // reset
    reset()
        .or_else([this](auto& e) {
            RCLCPP_ERROR(node_->get_logger(), toString(e));
        });

    const geometry_msgs::msg::TwistStamped empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist));

    // Fill last two commands with default constructed commands
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    return SuccessT{};
}

ReturnT OmniDriveControl::update(const rclcpp::Time & time,
                                 const OmniWheelStates& wheel_states,
                                 const std::shared_ptr<geometry_msgs::msg::Twist> cmd)
{
    // Write fake header in the stored stamped command
    std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd_stamped;
    cmd_stamped->twist = *cmd;
    cmd_stamped->header.stamp = node_->get_clock()->now();
    received_velocity_msg_ptr_.set(std::move(cmd_stamped));
    return update(time, wheel_states);
}

ReturnT OmniDriveControl::update(const rclcpp::Time & time,
                                    const OmniWheelStates& wheel_states)
{
    auto logger = node_->get_logger();

    std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr) {
        RCLCPP_ERROR(logger, "Velocity message received was a nullptr.");
        return  ErrorT(ErrorEnum::UNKNOWN_ERROR);
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_) {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.linear.y = 0.0;
        last_command_msg->twist.angular.z = 0.0;
    }

    previous_update_timestamp_ = time;

    // todo flags
    bool simulation = false;
    bool estop = false;

    for (int i=0; i<NUM_WHEELS; i++){
        // update with current feedback
        wheels_[i]->update(wheel_states[i].vel, wheel_states[i].rot);
        // set latest command
        wheels_[i]->command(*last_command_msg, simulation, estop);
    }

    odometry_.update(time);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    if (previous_publish_timestamp_ + publish_period_ < time) {
        previous_publish_timestamp_ += publish_period_;

        if (realtime_odometry_publisher_->trylock()) {
            auto & odometry_message = realtime_odometry_publisher_->msg_;
            odometry_message.header.stamp = time;
            odometry_message.pose.pose.position.x = odometry_.getX();
            odometry_message.pose.pose.position.y = odometry_.getY();
            odometry_message.pose.pose.orientation.x = orientation.x();
            odometry_message.pose.pose.orientation.y = orientation.y();
            odometry_message.pose.pose.orientation.z = orientation.z();
            odometry_message.pose.pose.orientation.w = orientation.w();
            odometry_message.twist.twist.linear.x = odometry_.getLinearX();
            odometry_message.twist.twist.linear.y = odometry_.getLinearY();
            odometry_message.twist.twist.angular.z = odometry_.getAngular();
            realtime_odometry_publisher_->unlockAndPublish();
        }

        if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock()) {
            auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
            transform.header.stamp = time;
            transform.transform.translation.x = odometry_.getX();
            transform.transform.translation.y = odometry_.getY();
            transform.transform.rotation.x = orientation.x();
            transform.transform.rotation.y = orientation.y();
            transform.transform.rotation.z = orientation.z();
            transform.transform.rotation.w = orientation.w();
            realtime_odometry_transform_publisher_->unlockAndPublish();
        }
    }

    return SuccessT{};
}

OmniWheelStates OmniDriveControl::getCtrlCmd() const
{
    OmniWheelStates cmd;
    for (int i=0; i<NUM_WHEELS; i++){
        cmd[i].rot = wheels_[i]->getAngleCmd();
        cmd[i].vel = wheels_[i]->getVelocityCmd();
    }
    return cmd;
}

geometry_msgs::msg::Twist OmniDriveControl::getVelocityState() const
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = odometry_.getLinearX();
    twist.linear.y = odometry_.getLinearY();
    twist.angular.z = odometry_.getAngular();
    return twist;
}

ReturnT OmniDriveControl::reset()
{
    odometry_.resetOdometry();

    // release the old queue
    std::queue<geometry_msgs::msg::TwistStamped> empty;
    std::swap(previous_commands_, empty);

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;

    return SuccessT{};
}

ReturnT OmniDriveControl::halt()
{
    // set velocity to zero
    // do not alter steering
    for (int i=0; i<NUM_WHEELS; i++){
        wheels_[i]->brake();
    }
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(vifware_vehicle_interfaces::OmniDriveControl)
