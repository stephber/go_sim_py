/*
 * File: iir_flt.cpp
 * Created Date: Tuesday, August 6th 2024, 13:11:06
 * Author: Martin Kirchengast
 * -----
 * Copyright (c) 2024 Virtual Vehicle Research GmbH
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	----------------------------------------------------------
 */
#include <cmath>
#include <vifware_vehicle_interface_common/filters/iir_flt.hpp>

vifware_vehicle_interfaces::IIRFilter::IIRFilter(const std::vector<double> &A, const std::vector<double> &B, const std::vector<double> &C, const double D)
    : A_(A), B_(B), C_(C), D_(D)
{
    n_ = static_cast<uint64_t>(sqrt(A.size()));
    x_.resize(n_, 0);
}

double vifware_vehicle_interfaces::IIRFilter::compute(const double val)
{
    // compute filtered output
    double yFlt = C_[0] * x_[0] + D_ * val;

    for (size_t i = 1; i < n_; i++)
        yFlt += C_[i] * x_[i];

    // update internal state vector
    const auto xOld = x_;

    for (size_t r = 0; r < n_; r++)
    {
        x_[r] = A_[r * n_] * xOld[0] + B_[r] * val;

        for (size_t c = 1; c < n_; c++)
            x_[r] += A_[r * n_ + c] * xOld[c];
    }

    return yFlt;
}