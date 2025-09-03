#pragma once

#include <autoware_interface/autoware_interface.hpp>

#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>

namespace vifware_vehicle_interfaces {

    int toAutowareGearReport(GearStatus g)
    {
        using AutowareGear = autoware_vehicle_msgs::msg::GearReport;
        if (g == GearStatus::PARK) {
            return static_cast<int>(AutowareGear::PARK);
        } else if (g == GearStatus::REVERSE) {
            return static_cast<int>(AutowareGear::REVERSE);
        } else if (g == GearStatus::DRIVE) {
            return static_cast<int>(AutowareGear::DRIVE);
        } else if (g == GearStatus::LOW) {
            return static_cast<int>(AutowareGear::LOW);
        } else if (g == GearStatus::NEUTRAL) {
            return static_cast<int>(AutowareGear::NEUTRAL);
        } else {
            return static_cast<int>(AutowareGear::NONE);
        }
    }

    GearStatus autowareGearCommandToGearStatus(int g)
    {
        if (g == 0) {
            return  GearStatus::NONE;
        } else if (g == 1) {
            return GearStatus::NEUTRAL;
        } else if (g > 1 && g < 20) {
            return GearStatus::DRIVE;
        } else if (g == 20 || g == 21) {
            return GearStatus::REVERSE;
        } else if (g == 22) {
            return GearStatus::PARK;
        } else if (g == 23 || g == 24) {
            return GearStatus::LOW;
        } else {
            return GearStatus::NONE;
        }
    }

    int toAutowareTurnIndicatorsReport(TurnSignal t)
    {
        using AutowareTurn = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
        if (t == TurnSignal::RIGHT) {
            return static_cast<int>(AutowareTurn::ENABLE_RIGHT);
        } else if (t == TurnSignal::LEFT) {
            return static_cast<int>(AutowareTurn::ENABLE_LEFT);
        } else if (t == TurnSignal::NONE) {
            return static_cast<int>(AutowareTurn::DISABLE);
        } else {
            return static_cast<int>(AutowareTurn::DISABLE);
        }
    }

    TurnSignal autowareTurnIndicatorsCommandToTurnSignal(int t)
    {
        if (t == 0 || t == 1) {
            return  TurnSignal::NONE;
        } else if (t == 2) {
            return TurnSignal::LEFT;
        } else if (t == 3) {
            return TurnSignal::RIGHT;
        } else {
            return TurnSignal::NONE;
        }
    }

    int toAutowareHazardLightsReport(TurnSignal t)
    {
        using AutowareHazardLight = autoware_vehicle_msgs::msg::HazardLightsReport;

        if (t == TurnSignal::HAZARD) {
            return static_cast<int>(AutowareHazardLight::ENABLE);
        } else {
            return static_cast<int>(AutowareHazardLight::DISABLE);
        }
    }

    TurnSignal autowareHazardLightsCommandToTurnSignal(int hl)
    {
        if (hl == 0 || hl == 1) {
            return  TurnSignal::NONE;
        } else if (hl == 2) {
            return TurnSignal::HAZARD;
        } else {
            return TurnSignal::NONE;
        }
    }

    int toAutowareControlMode(ControlMode m)
    {
        using AutowareControlMode = autoware_vehicle_msgs::msg::ControlModeReport;
        if (m == ControlMode::NONE) {
            return static_cast<int>(AutowareControlMode::NO_COMMAND);
        } else if (m == ControlMode::AUTONOMOUS) {
            return static_cast<int>(AutowareControlMode::AUTONOMOUS);
        } else if (m == ControlMode::MANUAL) {
            return static_cast<int>(AutowareControlMode::MANUAL);
        } else if (m == ControlMode::ERROR) {
            return static_cast<int>(AutowareControlMode::NOT_READY);
        } else {
            return static_cast<int>(AutowareControlMode::NO_COMMAND);
        }
    }

} //end namespace
