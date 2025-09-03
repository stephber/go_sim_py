/*
 * File: iir_flt.hpp
 * Created Date: Friday, July 26th 2024, 15:22:29
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
    class IIRFilter : ScalarFilter
    {
    public:
        explicit IIRFilter(const std::vector<double> &A, const std::vector<double> &B, const std::vector<double> &C, const double D);
        double compute(const double val) override;
    
    protected:
        std::vector<double> A_, B_, C_, x_;
        double D_;
        uint64_t n_;
    };
}