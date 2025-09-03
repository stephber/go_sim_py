# vifware_vehicle_interfaces / autoware_interface

This ROS 2 component (`autoware_interface`) acts as an interface between an Autoware stack and a vehicle platform. It subscribes to command topics from Autoware and publishes status information back to Autoware using shared memory

## Overview

The `autoware_interface` node facilitates the communication of control commands and vehicle state between an autonomous driving system (like Autoware) and the underlying vehicle hardware. It handles the necessary message conversions and topic remapping.

## Features

* **Subscribes to Autoware Command Topics:**
* **Publishes Vehicle Status Topics to Autoware:**
* **Message Conversion:** Handles the conversion between Autoware message types and the internal representation used within the vehicle interface.
* **Parameter for Base Frame ID:** Allows configuration of the base frame ID for the published velocity status.

## Parameters

* **`base_frame_id` (string, default: "base_link"):** The frame ID to be used in the header of the `/vehicle/status/velocity_status` messages.

## Subscriber
* `/control/command/control_cmd` (`autoware_auto_control_msgs::msg::AckermannControlCommand`): Receives desired acceleration, velocity, and steering angle commands.
* `/control/command/gear_cmd` (`autoware_auto_vehicle_msgs::msg::GearCommand`): Receives desired gear commands (e.g., forward, reverse, neutral).
* `/control/command/turn_indicators_cmd` (`autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`): Receives desired turn indicator commands (e.g., left, right).
* `/control/command/hazard_lights_cmd` (`autoware_auto_vehicle_msgs::msg::HazardLightsCommand`): Receives desired hazard lights commands (enable/disable).
* `/vehicle/engage` (`std_msgs::msg::Bool`): Receives the engage/disengage command for autonomous driving.
* `/control/command/emergency_cmd` (`std_msgs::msg::Bool`): Receives an emergency stop command.

## Publisher
* `/vehicle/status/control_mode` (`autoware_auto_vehicle_msgs::msg::ControlModeReport`): Publishes the current control mode of the vehicle.
* `/vehicle/status/velocity_status` (`autoware_auto_vehicle_msgs::msg::VelocityReport`): Publishes the current longitudinal and angular velocity of the vehicle.
* `/vehicle/status/steering_status` (`autoware_auto_vehicle_msgs::msg::SteeringReport`): Pubishes the current steering tire angle.
* `/vehicle/status/gear_status` (`autoware_auto_vehicle_msgs::msg::GearReport`): Publishes the current gear status of the vehicle.
* `/vehicle/status/turn_indicators_status` (`autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport`): Publishes the current turn indicator status.
* `/vehicle/status/hazard_lights_status` (`autoware_auto_vehicle_msgs::msg::HazardLightsReport`): Publishes the current hazard lights status.
* `/vehicle/status/actuation_status` (`autoware_auto_vehicle_msgs::msg::ActuationStatus`): Publishes the status of actuators (e.g., acceleration, brake, steering).
* `/vehicle/status/steering_wheel_status` (`std_msgs::msg::Float32`): Publishes the current steering wheel angle.

## License
Copyright Virtual Vehicle Research GmbH, 2025