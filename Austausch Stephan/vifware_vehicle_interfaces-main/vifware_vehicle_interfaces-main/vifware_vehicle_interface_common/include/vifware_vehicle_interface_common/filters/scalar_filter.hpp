/*
 * File: scalar_filter.hpp
 * Created Date: Friday, July 26th 2024, 15:17:40
 * Author: Martin Kirchengast
 * -----
 * Copyright (c) 2024 Virtual Vehicle Research GmbH
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	----------------------------------------------------------
 */
#pragma once

namespace vifware_vehicle_interfaces
{
    class ScalarFilter
    {
    public:
        virtual double compute(const double val) = 0;
    };
}