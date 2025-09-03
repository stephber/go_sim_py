#pragma once

namespace spider_path_tracking_manager
{

    enum class PtfErrorEnum
        {
            NO_ERROR,
            UNKNOWN_ERROR,
            ROS_COMM_ERROR,
            PARAMETER_ERROR,
            PLANNER_ERROR,
            CONTROLLER_ERROR,
            FSM_ERROR
        };

    constexpr const char* toString(PtfErrorEnum e) noexcept
    {
        switch (e) {
        case PtfErrorEnum::NO_ERROR: return "PTF no error";
        case PtfErrorEnum::UNKNOWN_ERROR: return "PTF unknown error";
        case PtfErrorEnum::ROS_COMM_ERROR: return "PTF ros communication error";
        case PtfErrorEnum::PARAMETER_ERROR: return "PTF parameter error";
        case PtfErrorEnum::PLANNER_ERROR: return "PTF planner error";
        case PtfErrorEnum::CONTROLLER_ERROR: return "PTF controller error";
        case PtfErrorEnum::FSM_ERROR: return "PTF finite state machine error";
        }
        return "undefined error";
    }

} // end namespace
