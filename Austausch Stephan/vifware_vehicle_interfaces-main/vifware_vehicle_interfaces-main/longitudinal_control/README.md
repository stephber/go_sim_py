# vifware_vehicle_interfaces / longitudinal_control
This ROS 2 component (`longitudinal_control`) provides longitudinal control functionalities for a vehicle. 

## Overview

The `longitudinal_control` node takes desired acceleration or velocity commands as input (depending on the active control mode) and outputs a torque command for the vehicle's powertrain. It incorporates a PID-like controller with feedforward terms to achieve accurate and responsive longitudinal motion.

## Parameters
Configuration file loading is handled via by the main node, e.g. via vifware_torus_interface.

## License
Copyright Virtual Vehicle Research GmbH, 2025