/*
 * File: lin_vel_obs.cpp
 * Created Date: Tuesday, August 6th 2024, 17:05:59
 * Author: Martin Kirchengast
 * -----
 * Copyright (c) 2024 Virtual Vehicle Research GmbH
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	----------------------------------------------------------
 */
#include <cmath>
#include <vifware_vehicle_interface_common/filters/lin_vel_obs.hpp>

vifware_vehicle_interfaces::LinVelObserver::LinVelObserver(const std::vector<double> &A, const std::vector<double> &B, const std::vector<double> &L, const uint64_t vIdx) 
    : A_(A), B_(B), L_(L), vIdx_(vIdx)
{
    n_ = static_cast<uint64_t>(sqrt(A.size()));
    m_ = static_cast<uint64_t>(B.size() / n_);
    x_.resize(n_, 0);
}

double vifware_vehicle_interfaces::LinVelObserver::compute(const std::vector<double> &u, const double y)
{
    const double yFlt = x_[vIdx_];

    // check dimension mismatch
    if (u.size() == m_)
    {
        // update internal state vector
        const double err = y - yFlt;
        const auto xOld = x_;

        for (size_t r = 0; r < n_; r++)
        {
            x_[r] = A_[r * n_] * xOld[0] + B_[r * m_] * u[0] + L_[r] * err;

            for (size_t cA = 1; cA < n_; cA++)
                x_[r] += A_[r * n_ + cA] * xOld[cA];
            for (size_t cB = 1; cB < m_; cB++)
                x_[r] += B_[r * m_ + cB] * u[cB];
        }
    }

    return yFlt;
}