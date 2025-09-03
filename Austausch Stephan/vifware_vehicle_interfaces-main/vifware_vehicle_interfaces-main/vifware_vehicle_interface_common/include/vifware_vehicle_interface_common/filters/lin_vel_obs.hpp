/*
 * File: lin_vel_obs.hpp
 * Created Date: Friday, July 26th 2024, 15:23:53
 * Author: Martin Kirchengast
 * -----
 * Copyright (c) 2024 Virtual Vehicle Research GmbH
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	----------------------------------------------------------
 */
#pragma once

#include <stdint.h>
#include <vector>

namespace vifware_vehicle_interfaces
{
    class LinVelObserver
    {
    public:
        explicit LinVelObserver(const std::vector<double> &A, const std::vector<double> &B, const std::vector<double> &L, const uint64_t vIdx);
        double compute(const std::vector<double> &u, const double y);

    protected:
        std::vector<double> A_, B_, L_, x_;
        uint64_t m_, n_, vIdx_;
    };
}