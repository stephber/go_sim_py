// Copyright VIF
#include <vifware_unitree_go1_driver/unitree_go1_driver.hpp>

using namespace vifware_unitree_go1_interface;

UnitreeGo1Driver::UnitreeGo1Driver(const rclcpp::NodeOptions & options)
{
    node_ = std::make_shared<rclcpp::Node>("unitree_go1_driver", options);

    initComm();
    initParams();
    initTimer();

    // init high cmd -- Default command values
    high_cmd_.head[0] = 0xFE; //??
    high_cmd_.head[1] = 0xEF; //??
    high_cmd_.level_flag = 0xEE; //from unitree_legged_sdk, HIGHLEVEL TODO - move to UDP node?
    high_cmd_.mode = to_value(Go1Mode::idle);
    high_cmd_.gait_type = to_value(Go1Gait::idle);
    high_cmd_.speed_level = to_value(Go1SpeedLevel::low);
    high_cmd_.velocity[0] = 0.0;
    high_cmd_.velocity[1] = 0.0;
    high_cmd_.yaw_speed = 0.0;
    // default walking gait is trotting
    walking_gait_ = Go1Gait::trot;


    // components
    ad_client_ = std::make_shared<vifware_vehicle_interfaces::AutowareAdApiClient>(node_);
    aw_if_ = std::make_shared<vifware_vehicle_interfaces::AutowareInterface>(node_);
}

// implement component
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
UnitreeGo1Driver::get_node_base_interface() const
{
  return this->node_->get_node_base_interface();
}

void UnitreeGo1Driver::initComm()
{
    high_state_sub_ = node_->create_subscription<ros2_unitree_legged_msgs::msg::HighState>("/vehicle/high_state", 1,
       [this](const ros2_unitree_legged_msgs::msg::HighState::ConstSharedPtr msg) {
            high_state_ = msg;
       });
    high_cmd_pub_ = this->node_->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("/vehicle/high_cmd", 1);

    // // internal
    // autoware_if_control_command_sub_ =
    //     this->node_->create_subscription<UnitreeGo1Driver::AutowareIfControlCommand>("/vehicle/vif_internal/autoware_control_command", 1,
    //                    [this](const UnitreeGo1Driver::AutowareIfControlCommand::ConstSharedPtr msg) {
    //                        aw_ctrl_cmd_ = msg;
    //                    });

    // autoware_if_misc_command_sub_ =
    //     this->node_->create_subscription<UnitreeGo1Driver::AutowareIfMiscCommand>("/vehicle/vif_internal/autoware_misc_command", 1,
    //                                                                  [this](const UnitreeGo1Driver::AutowareIfMiscCommand::ConstSharedPtr msg) {
    //                                                                      aw_misc_cmd_ = msg;
    //                                                                  });
    // // publisher
    // autoware_if_report_pub_ =
    //     this->node_->create_publisher<UnitreeGo1Driver::AutowareIfReport>("/vehicle/vif_internal/autoware_report", 1);

    autoware_imu_pub_ =
        this->node_->create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/imu_data", 1);

    autoware_twist_pub_ =
        this->node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/sensing/vehicle_velocity_converter/twist_with_covariance", 1);

    //Services
    srv_reset_state_ = this->node_->create_service<std_srvs::srv::Empty>("reset_state",
                                                            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                reset_cmd_vel();
                                                                reset_state();
                                                            });
    srv_stand_up_ = this->node_->create_service<std_srvs::srv::Empty>("stand_up",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::position_stand_up);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });
    srv_recover_stand_ = this->node_->create_service<std_srvs::srv::Empty>("recover_stand",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::recovery_stand);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });

    srv_lay_down_ = this->node_->create_service<std_srvs::srv::Empty>("lay_down",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::position_stand_down);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });

    srv_damping_ = this->node_->create_service<std_srvs::srv::Empty>("damping",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::damping);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });

    srv_jump_yaw_ = this->node_->create_service<std_srvs::srv::Empty>("jump_yaw",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::jump_yaw);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });

    srv_beg_ = this->node_->create_service<std_srvs::srv::Empty>("beg",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::straight_hand);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });

    srv_dance1_ = this->node_->create_service<std_srvs::srv::Empty>("dance1",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::dance1);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });

    srv_dance2_ = this->node_->create_service<std_srvs::srv::Empty>("dance2",
                                                         [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                                                                 reset_cmd_vel();
                                                                 high_cmd_.mode = to_value(Go1Mode::dance2);
                                                                 reset_state(); //Reset state after command is sentreset_cmd_vel();
                                                         });
}

void UnitreeGo1Driver::initParams()
{
    loop_rate_hz_ = this->node_->declare_parameter("loop_rate_hz", 50.0);
    wheelbase_ = this->node_->declare_parameter("wheelbase", 0.5);
}

void UnitreeGo1Driver::initTimer()
{
    // loop timer
    const auto loop_period_ns = rclcpp::Rate(loop_rate_hz_).period();
    loop_timer_ = rclcpp::create_timer(this->node_, this->node_->get_clock(), loop_period_ns,
                                       [this]() {
                                           timerCallback();
                                       });
}

void UnitreeGo1Driver::reset_cmd_vel() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.0;
}

void UnitreeGo1Driver::reset_state() {
    reset_state_ = true;
    reset_counter_ = 0;
}

void UnitreeGo1Driver::timerCallback()
{
    // forward command from autoware
    // if (aw_ctrl_cmd_ && aw_misc_cmd_) {
    //     if (aw_misc_cmd_->gear_cmd.data == vifware_vehicle_interface_msgs::msg::GearStatus::DRIVE ||
    //         vifware_vehicle_interface_msgs::msg::GearStatus::REVERSE) {
            // double steer_angle = aw_ctrl_cmd_->steering_tire_angle; // * M_PI / 180.0; // [rad]
            // cmd_vel_.linear.x = linear;
            // cmd_vel_.angular.z = std::tan(steer_angle) * linear / wheelbase_;

    if (aw_if_->data_from_autoware_.gear_status_ == vifware_vehicle_interfaces::GearStatus::DRIVE ||
        aw_if_->data_from_autoware_.gear_status_ == vifware_vehicle_interfaces::GearStatus::REVERSE ) {

        double linear = aw_if_->data_from_autoware_.ctrl_cmd_speed_;
        double steer_angle = aw_if_->data_from_autoware_.ctrl_cmd_steering_tire_angle_; // [rad]
        cmd_vel_.linear.x = linear;
        cmd_vel_.angular.z = std::tan(steer_angle) * linear / wheelbase_;

        //set mode and gait type
        high_cmd_.mode = to_value(Go1Mode::target_velocity_walking);
        high_cmd_.gait_type = to_value(walking_gait_);





    } else {
        reset_cmd_vel();
    }

    high_cmd_.velocity[0] = cmd_vel_.linear.x;
    high_cmd_.velocity[1] = cmd_vel_.linear.y;
    high_cmd_.yaw_speed = cmd_vel_.angular.z;

    high_cmd_pub_->publish(high_cmd_);


    //Reset state to idle after one message is sent for anything but walking
    if (reset_state_) {
      if (reset_counter_ >= 10) {
        high_cmd_.mode = to_value(Go1Mode::idle);
        reset_state_ = false;
      }
      reset_counter_++;
    }

    publishToAutowareInterface();
}


void UnitreeGo1Driver::publishToAutowareInterface()
{
    // vifware_vehicle_interface_msgs::msg::AutowareInterfaceReport rpt;
    // rpt.stamp = node_->get_clock()->now();

    // todo check if Autoware send Neutral inbetween switching form D to R
    data_to_autoware_.gear_status_ = aw_if_->data_from_autoware_.gear_status_;

    // (0 = None, 1 = Autonomous, 2 = Manual, 3 = Error)
    //rpt.control_mode.data = 1; // TODO: fill from FSM
    data_to_autoware_.ctrl_mode_ = vifware_vehicle_interfaces::ControlMode::AUTONOMOUS;

    double vel { 0. };
    double yawrate { 0. };
    if (high_state_) {
      vel = high_state_->velocity[0];
      yawrate = high_state_->yaw_speed;
    }
    //rpt.vel_linear = vel;
    //rpt.vel_angular = yawrate;
    data_to_autoware_.vel_lin_ =  vel;
    data_to_autoware_.vel_ang_ =  yawrate;

    if (vel > 0.01) {
        // rpt.steering_tire_angle = std::atan2(yawrate * wheelbase_, vel );
        // // same value for steering wheel?
        // rpt.steering_wheel_angle = rpt.steering_tire_angle;
        data_to_autoware_.steering_tire_angle_ = std::atan2(yawrate * wheelbase_, vel ) * M_PI / 180;
        // same value for steering wheel?
        data_to_autoware_.steering_wheel_angle_ = data_to_autoware_.steering_tire_angle_;
    }

    // rpt.accel_en = true; // TODO: fill from FSM
    // rpt.brake_en = true; // TODO: fill from FSM
    // rpt.steer_en = true; // TODO: fill from FSM
    // autoware_if_report_pub_->publish(rpt);
    data_to_autoware_.actuation_status_accel_ = true; // TODO: fill from FSM
    data_to_autoware_.actuation_status_brake_ = true; // TODO: fill from FSM
    data_to_autoware_.actuation_status_steer_ = true; // TODO: fill from FSM
    aw_if_->publish(data_to_autoware_);

    // extract values from high state for twist / imu topics
     if (high_state_) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_data;
        twist_data.header.frame_id = "base_link";
        twist_data.header.stamp = node_->get_clock()->now();
        twist_data.twist.twist.linear.x = high_state_->velocity[0];
        twist_data.twist.twist.linear.y = high_state_->velocity[1];
        twist_data.twist.twist.linear.z = high_state_->velocity[2];
        twist_data.twist.twist.angular.x = 0.0;  //high_state_->imu.rpy[0];
        twist_data.twist.twist.angular.y = 0.0;  //high_state_->imu.rpy[1];
        twist_data.twist.twist.angular.z = high_state_->yaw_speed; // high_state_->imu.rpy[2];
        twist_data.twist.covariance[0] = 0.1;  // Linear X
        twist_data.twist.covariance[7] = 0.1;  // Linear Y
        twist_data.twist.covariance[14] = 0.1; // Linear Z
        twist_data.twist.covariance[21] = 0.1; // Angular X
        twist_data.twist.covariance[28] = 0.1; // Angular Y
        twist_data.twist.covariance[35] = 0.1; // Angular Z

        sensor_msgs::msg::Imu imu_data;
        imu_data.header = twist_data.header;
        imu_data.orientation.x = high_state_->imu.quaternion[0];
        imu_data.orientation.y = high_state_->imu.quaternion[1];
        imu_data.orientation.z = high_state_->imu.quaternion[2];
        imu_data.orientation.w = high_state_->imu.quaternion[3];
        imu_data.orientation_covariance[0] = 0.1;
        imu_data.orientation_covariance[4] = 0.1;
        imu_data.orientation_covariance[8] = 0.1;
        imu_data.angular_velocity.x = 0.0;  //high_state_->imu.rpy[0];
        imu_data.angular_velocity.y = 0.0; //high_state_->imu.rpy[1];
        imu_data.angular_velocity.z = high_state_->yaw_speed; //high_state_->imu.rpy[2];
        imu_data.angular_velocity_covariance[0] = 0.1;
        imu_data.angular_velocity_covariance[4] = 0.1;
        imu_data.angular_velocity_covariance[8] = 0.1;
        imu_data.linear_acceleration.x = high_state_->imu.accelerometer[0];
        imu_data.linear_acceleration.y = high_state_->imu.accelerometer[1];
        imu_data.linear_acceleration.z = high_state_->imu.accelerometer[2];
        imu_data.linear_acceleration_covariance = imu_data.angular_velocity_covariance;
              
        // publish twist / imu values
        autoware_imu_pub_->publish(imu_data);
        autoware_twist_pub_->publish(twist_data);
     }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vifware_unitree_go1_interface::UnitreeGo1Driver)
