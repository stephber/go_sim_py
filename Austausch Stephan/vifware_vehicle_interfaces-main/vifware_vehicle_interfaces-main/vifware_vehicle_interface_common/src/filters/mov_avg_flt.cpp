/*
 * File: mov_avg_flt.cpp
 * Created Date: Friday, July 26th 2024, 15:30:41
 * Author: Martin Kirchengast
 * -----
 * Copyright (c) 2024 Virtual Vehicle Research GmbH
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	----------------------------------------------------------
 */
#include <vifware_vehicle_interface_common/filters/mov_avg_flt.hpp>

vifware_vehicle_interfaces::MovAvgFilter::MovAvgFilter(const uint32_t len) : bufferIdx_(0), length_(len)
{
    buffer_.resize(len, 0);
}

double vifware_vehicle_interfaces::MovAvgFilter::compute(const double val)
{
    // store current input signal
    buffer_[bufferIdx_] = val;

    // sum up current and previous values
    double sum = val;

    for (std::size_t i = 1; i < length_; i++)
    {
        // negative indices are counted from array end 
        const std::size_t curIdx = (bufferIdx_ - i + length_) % length_;
        sum += buffer_[curIdx];
    }

    // increase buffer index and compute result
    bufferIdx_ = (bufferIdx_ + 1) % length_;
    return sum / length_;
}