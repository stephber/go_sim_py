// Copyright VIF

#pragma once

#include<memory>

namespace vifware_unitree_go1_interface {

//TODO move into a shared library
enum class Go1Mode : uint8_t
{
   idle = 0
  ,force_stand = 1
  ,target_velocity_walking = 2
  ,target_position_walking = 3
  ,path_walking = 4
  ,position_stand_down = 5
  ,position_stand_up = 6
  ,damping = 7
  ,recovery_stand = 8
  ,backflip = 9
  ,jump_yaw = 10
  ,straight_hand = 11
  ,dance1 = 12
  ,dance2 = 13
};

enum class Go1Gait : uint8_t
{
   idle = 0
  ,trot = 1
  ,trot_running = 2
  ,climb_stairs = 3
  ,trot_obstacle = 4
};

enum class Go1SpeedLevel : uint8_t
{
   low = 0
  ,medium = 1
  ,high = 2
};

//https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template <typename Enumeration>
auto to_value(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

} // end namespace
