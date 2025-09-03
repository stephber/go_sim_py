/*
 * File: mov_avg_flt.hpp
 * Created Date: Friday, July 26th 2024, 15:20:11
 * Author: Martin Kirchengast
 * -----
 * Copyright (c) 2024 Virtual Vehicle Research GmbH
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	----------------------------------------------------------
 */
#pragma once

#include <vifware_vehicle_interface_common/filters/scalar_filter.hpp>
#include <stdint.h>
#include <vector>

namespace vifware_vehicle_interfaces
{
    class MovAvgFilter : ScalarFilter
    {
    public:
        explicit MovAvgFilter(const uint32_t len);
        double compute(const double val) override;

    protected:
        std::size_t bufferIdx_;
        std::vector<double> buffer_;
        uint32_t length_;
    };
}