// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "omnidrive_control/odometry.hpp"

using namespace vifware_vehicle_interfaces;

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(const rclcpp::Time & time)
{
  double curr_linear_x = 0.0;
  double curr_linear_y = 0.0;
  double curr_angular_z = 0.0;

  int i = 1;
  for (const std::shared_ptr<Wheel> & wheel : wheels_) {
    curr_linear_x += std::cos(wheel->getAngleFB()) * wheel->getVelocityFB();
    curr_linear_y += std::sin(wheel->getAngleFB()) * wheel->getVelocityFB();

    double sign = wheel->getInvertFactor();
    double sector = ((wheel->getOffsetX() > 0 && wheel->getOffsetY() < 0) ||
                     (wheel->getOffsetX() < 0 && wheel->getOffsetY() > 0))
                      ? M_PI_4
                      : -M_PI_4;

    auto tmp = sign * wheel->getVelocityFB() * std::cos(sector - wheel->getAngleFB()) /
               wheel->getCenterOffset();

    // curr_angular_z += 0;  // tmp;
    curr_angular_z += sign * wheel->getVelocityFB() * std::cos(sector - wheel->getAngleFB())
         / wheel->getCenterOffset();

    i++;
  }

  curr_linear_x /= wheels_.size();
  curr_linear_y /= wheels_.size();
  curr_angular_z /= wheels_.size();

  /*std::cout << std::endl << "=== sumed up vel ==="
            << std::endl << curr_linear_x
            << std::endl << curr_linear_y
            << std::endl << curr_angular_z
            << std::endl << "=== "
            << std::endl << std::endl;*/

  /// We cannot estimate the speed with very small time intervals:
  const double dt = (time - timestamp_).seconds();
  if (dt < 0.0001) {
    return false;  // Interval too small to integrate with
  }

  timestamp_ = time;

  /// Estimate speeds using a rolling mean to filter them out:
  linear_x_accumulator_.accumulate(curr_linear_x);
  linear_y_accumulator_.accumulate(curr_linear_y);
  angular_accumulator_.accumulate(curr_angular_z);

  // The rolling mean is the mean over the last N samples.
  linear_x_ = linear_x_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  double delta_lin_x = linear_x_ * dt;
  double delta_lin_y = linear_y_ * dt;

  x_ += (delta_lin_x * std::cos(heading_) - (delta_lin_y * std::sin(heading_)));
  y_ += (delta_lin_x * std::sin(heading_) + (delta_lin_y * std::cos(heading_)));

  heading_ += angular_ * dt;

  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::addWheel(std::shared_ptr<Wheel> wheel)
{
  wheels_.push_back(wheel);
}

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}
