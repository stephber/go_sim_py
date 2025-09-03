// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "omnidrive_control/wheel.hpp"

using namespace vifware_vehicle_interfaces;

Wheel::Wheel(rclcpp::Logger & logger) : logger_(logger)
{
}

bool Wheel::configure(
  std::string prefix, double max_rot_angle, double offset_x, double offset_y, double rot_factor,
  double invert)
{
  max_rot_angle_ = max_rot_angle;
  offset_x_ = offset_x;
  offset_y_ = offset_y;
  center_offset_ = std::hypot(offset_x_, offset_y_);
  rot_factor_ = rot_factor;
  invert_factor_ = invert;

  // todo error handling
  return true;
}

void Wheel::update(const double vel_fb, const double rot_fb)
{
  vel_fb_ = vel_fb;
  rot_fb_ = rot_fb;
}

void Wheel::command(Twist cmd, bool simulation, bool estop)
{
  // calculate target velocity ond rotation
  float v_x = cmd.twist.linear.x - cmd.twist.angular.z * offset_y_;
  float v_y = cmd.twist.linear.y + cmd.twist.angular.z * offset_x_;

  float v = std::hypot(v_x, v_y);
  float a = std::atan2(v_y, v_x);

  v = std::fabs(a) > max_rot_angle_ ? -v : v;

  // Check if the wheel allows an rotation angle, if not set it to zero
  if (std::abs(max_rot_angle_) < 0.001) {
    // set rotation to zero
    a = 0.0;
  } else {
    // limit to max rotation angle
    if (a > max_rot_angle_) {
      float flipped = a - M_PI;
      if (flipped < -max_rot_angle_ || flipped > max_rot_angle_) {
        a = getAngleFB();
        if (a > 0) v = -v;
        RCLCPP_ERROR_STREAM(
          logger_, "[omnidrive_control] Invalid taraget angle > max_rot_angle. "
                     << " Target: " << a);
      } else {
        a = flipped;
      }
    } else if (a < -max_rot_angle_) {
      float flipped = a + M_PI;
      if (flipped > max_rot_angle_ || flipped < -max_rot_angle_) {
        a = getAngleFB();
        if (a < 0) v = -v;
        RCLCPP_ERROR_STREAM(
          logger_, "[omnidrive_control] Invalid target angle < -max_rot_ange. "
                     << "Target: " << a);
      } else {
        a = flipped;
      }
    }
  }

  if (simulation) {
    v = -invert_factor_ * v;  // reverse wheel axis orientation
  }

  setAngle(-a);
  if (!estop)
    setVelocity(v);
  else
    setVelocity(0.0);
}

void Wheel::brake()
{
  vel_cmd_ = 0.;
}

void Wheel::setVelocity(double velocity)
{
  vel_cmd_ = velocity / rot_factor_;
}

double Wheel::getVelocityFB() const
{
  return vel_fb_ * rot_factor_ * invert_factor_;
}

double Wheel::getVelocityCmd() const
{
  return vel_cmd_;
}

void Wheel::setAngle(double angle)
{
  rot_cmd_ = angle;
}

double Wheel::getAngleFB() const
{
  return -rot_fb_;
}

double Wheel::getAngleCmd() const
{
  return rot_cmd_;
}

double Wheel::getInvertFactor() const
{
  return invert_factor_;
}

double Wheel::getCenterOffset() const
{
  return center_offset_;
}

double Wheel::getOffsetX() const
{
  return offset_x_;
}

double Wheel::getOffsetY() const
{
  return offset_y_;
}
