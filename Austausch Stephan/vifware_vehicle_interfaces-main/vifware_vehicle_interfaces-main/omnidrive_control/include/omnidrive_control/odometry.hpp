// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include "omnidrive_control/wheel.hpp"
#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace vifware_vehicle_interfaces
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);

  bool update(const rclcpp::Time & time);

  bool update(double left_pos, double right_pos, const rclcpp::Time & time);
  //  bool updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time & time);
  //  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);

  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

  // void setWheelParams(double wheel_separation, double left_wheel_radius, double
  // right_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  void addWheel(std::shared_ptr<Wheel> wheel);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_x_;  //   [m/s]
  double linear_y_;  //   [m/s]
  double angular_;   // [rad/s]

  // Wheel kinematic parameters [m]:
  std::vector<std::shared_ptr<Wheel>> wheels_;

  double wheel_separation_;
  double left_wheel_radius_;
  double right_wheel_radius_;

  //Previous wheel position/state [rad]:
  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // end namespace
