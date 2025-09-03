// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace vifware_vehicle_interfaces
{
/**
 * \brief Configuration and helper functions for a steered wheel
 */
class Wheel
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  Wheel(rclcpp::Logger & logger);

  rclcpp::Logger get_logger() const;

  bool configure(
    std::string prefix, double max_rot_angle, double offset_x, double offset_y, double rot_factor,
    double invert);

  void update(const double vel_fb, const double rot_fb);

  // calc  Calculate rotation and velocity of each wheel
  //  http://www.chiefdelphi.com/media/papers/download/2614
  void command(Twist cmd, bool simulation, bool estop = false);

  void brake();

  // get feedback
  double getAngleFB() const;
  double getVelocityFB() const;

  // get command
  double getAngleCmd() const;
  double getVelocityCmd() const;

  double getInvertFactor() const;
  double getCenterOffset() const;
  double getOffsetX() const;
  double getOffsetY() const;

private:
  void setVelocity(double velocity);
  void setAngle(double angle);

  double max_rot_angle_;
  double offset_x_;
  double offset_y_;
  double center_offset_;
  double rot_factor_{1.};
  double invert_factor_;  // invert velocity commands on one side of the vehicle

  double vel_cmd_{0.};
  double rot_cmd_{0.};
  double vel_fb_{0.};
  double rot_fb_{0.};

  rclcpp::Logger & logger_;
};

}  // namespace vifware_vehicle_interfaces
