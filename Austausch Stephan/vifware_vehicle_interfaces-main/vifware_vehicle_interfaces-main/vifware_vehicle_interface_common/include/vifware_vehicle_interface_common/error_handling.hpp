// Copyright (c) 2024, VIF
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include "iceoryx_hoofs/cxx/expected.hpp"

namespace vifware_vehicle_interfaces
{
    //enum class return_type : std::uint8_t { OK = 0, ERROR = 1 };
    // vifware vehicle interface error
    enum class ErrorEnum : std::uint8_t { NO_ERROR, UNKNOWN_ERROR, ROS_COMM_ERROR, PARAMETER_ERROR, FSM_ERROR, RUNTIME_ERROR };

constexpr const char * toString(ErrorEnum e) noexcept
{
  switch (e) {
    case ErrorEnum::NO_ERROR:
      return "no error";
    case ErrorEnum::UNKNOWN_ERROR:
      return "unknown error";
    case ErrorEnum::ROS_COMM_ERROR:
      return "ros communication error";
    case ErrorEnum::PARAMETER_ERROR:
      return "parameter error";
    case ErrorEnum::FSM_ERROR:
      return "finite state machine error";
  }
  return "undefined error";
}

    template<typename T>
    using ReturnValueT = iox::cxx::expected<T, ErrorEnum>;
    using ReturnT = iox::cxx::expected<void, ErrorEnum>;

    template<typename T>
    using SuccessValueT = iox::cxx::success<T>;
    using SuccessT = iox::cxx::success<void>;
    using ErrorT = iox::cxx::error<ErrorEnum>;

} // end namespace
