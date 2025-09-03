// Copyright VIF

#include <longitudinal_control/longitudinal_control.hpp>
#include <vifware_vehicle_interface_common/utils.hpp>

template <typename T> constexpr int sign(const T &val) { return (T(0) < val) - (val < T(0)); }

vifware_vehicle_interfaces::LongitudinalControl::LongitudinalControl(const std::shared_ptr<rclcpp::Node> &node, const double ctrlRate)
    : node_(node)
    , p_ctrl_rate_s_(ctrlRate)
{
#ifdef LON_CTRL_DBG_OUT
    dbg_pub_ = node_->create_publisher<LongitudinalControl::LonCtrlDbg>("/vehicle/vif_internal/lon_ctrl_dbg", 1);
#endif
    // parameters
    dynParamHandle_ = node_->add_on_set_parameters_callback(std::bind(&LongitudinalControl::cbParams, this, std::placeholders::_1));

    p_ctrl_mode_ = node_->declare_parameter(VIF_PARAM("ctrl_mode"), MODE_IDLE);

    p_ctrl0_lint_lim_ = node_->declare_parameter(VIF_PARAM("ctrl0_lint_lim"), 5.0);
    p_ctrl0_nlint_lim_ = node_->declare_parameter(VIF_PARAM("ctrl0_nlint_lim"), 2.0);
    p_ctrl0_ka_ = node_->declare_parameter(VIF_PARAM("ctrl0_ka"), 0.0);
    p_ctrl0_kp_ = node_->declare_parameter(VIF_PARAM("ctrl0_kp"), 4.0);
    p_ctrl0_ki_l_ = node_->declare_parameter(VIF_PARAM("ctrl0_ki_l"), 0.0);
    p_ctrl0_ki_nl_ = node_->declare_parameter(VIF_PARAM("ctrl0_ki_nl"), 0.0);
    p_ctrl0_kd_ = node_->declare_parameter(VIF_PARAM("ctrl0_kd"), 0.0);
    p_ctrl0_tc_em_ff_ = node_->declare_parameter(VIF_PARAM("ctrl0_tc_em_ff"), 0.043);
    p_ctrl0_w_ff_ = node_->declare_parameter(VIF_PARAM("ctrl0_w_ff"), 1.0);

    p_ts_a_acc_ = node_->declare_parameter(VIF_PARAM("ts_a_acc"), 1.0);
    p_ts_a_dec_ = node_->declare_parameter(VIF_PARAM("ts_a_dec"), -1.0);
    p_ts_v_ = node_->declare_parameter(VIF_PARAM("ts_v"), 0.0);
    p_ts_trq_ = node_->declare_parameter(VIF_PARAM("ts_trq"), 0.0);
    p_ts_t_delay_ = node_->declare_parameter(VIF_PARAM("ts_t_delay"), 1.0);

    p_trq_max_ = node_->declare_parameter(VIF_PARAM("trq_max"), 100.0);
    p_trq_min_ = node_->declare_parameter(VIF_PARAM("trq_min"), -100.0);

    p_veh_cr_ = node_->declare_parameter(VIF_PARAM("veh_cr"), 0.013);
    p_veh_cw_ = node_->declare_parameter(VIF_PARAM("veh_cw"), 0.25);
    p_veh_front_A_ = node_->declare_parameter(VIF_PARAM("veh_front_A"), 1.8);
    p_veh_i_fd_ = node_->declare_parameter(VIF_PARAM("veh_i_fd"), 9.59);
    p_veh_mass_ = node_->declare_parameter(VIF_PARAM("veh_mass"), 900.0);
    p_veh_r_wheel_ = node_->declare_parameter(VIF_PARAM("veh_r_wheel"), 0.298);
}

rcl_interfaces::msg::SetParametersResult vifware_vehicle_interfaces::LongitudinalControl::cbParams(const std::vector<rclcpp::Parameter> &parameters)
{
    for (rclcpp::Parameter const &param : parameters)
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            // controller 0 parameters
            if (param.get_name() == VIF_PARAM("ctrl0_lint_lim"))
                p_ctrl0_lint_lim_ = param.as_double();
            else if(param.get_name() == VIF_PARAM("ctrl0_nlint_lim"))
                p_ctrl0_nlint_lim_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_ka"))
                p_ctrl0_ka_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_kd"))
                p_ctrl0_kd_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_ki_l"))
                p_ctrl0_ki_l_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_ki_nl"))
                p_ctrl0_ki_nl_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_kp"))
                p_ctrl0_kp_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_w_ff"))
                p_ctrl0_w_ff_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ctrl0_tc_em_ff"))
                p_ctrl0_tc_em_ff_ = param.as_double();
            // test signal parameters
            else if (param.get_name() == VIF_PARAM("ts_a_acc"))
                p_ts_a_acc_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ts_a_dec"))
                p_ts_a_dec_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ts_v"))
                p_ts_v_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ts_trq"))
                p_ts_trq_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("ts_t_delay"))
                p_ts_t_delay_ = param.as_double();
            // torque limits
            else if (param.get_name() == VIF_PARAM("trq_max"))
                p_trq_max_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("trq_min"))
                p_trq_min_ = param.as_double();
            // vehicle parameters
            else if (param.get_name() == VIF_PARAM("veh_cr"))
                p_veh_cr_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("veh_cw"))
                p_veh_cw_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("veh_front_A"))
                p_veh_front_A_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("veh_i_fd"))
                p_veh_i_fd_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("veh_mass"))
                p_veh_mass_ = param.as_double();
            else if (param.get_name() == VIF_PARAM("veh_r_wheel"))
                p_veh_r_wheel_ = param.as_double();
        }
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER && param.get_name() == VIF_PARAM("ctrl_mode"))
        {
            auto new_ctrl_mode =  param.as_int();
            // possible control modes: 0, 1, 2, 3
            new_ctrl_mode = (new_ctrl_mode < MODE_IDLE) ? MODE_IDLE : ((new_ctrl_mode > MODE_CL_EXT_REF) ? MODE_CL_EXT_REF : new_ctrl_mode);

            // reset test signal start time if we switch control mode
            if (new_ctrl_mode && new_ctrl_mode != p_ctrl_mode_)
            {
                ts_phase_ = 0;
                ts_phase_start_ = node_->now();
            }

            p_ctrl_mode_ = new_ctrl_mode;
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

double vifware_vehicle_interfaces::LongitudinalControl::calc(LonCtrlInput &input)
{
    // controller output
    double torque = 0;

    // select velocity signal for controller
    const double v_veh = input.curVel_;

    switch (active_ctrl_mode_)
    {
    case MODE_IDLE:
        input.refAcc_ = 0;
        input.refVel_ = 0;
        torque = 0;
        // reset controller working set
        a_km1_ = 0;
        e_km1_ = 0;
        u_km1_ = 0;
        u_lint_km1_ = 0;
        u_nlint_km1_ = 0;
        u_sat_km1_ = 0;
#ifdef LON_CTRL_DBG_OUT
        dbg_.err_vel = 0;
        dbg_.trq_fb_int = 0;
        dbg_.trq_fb = 0;
#endif
        break;

    case MODE_OL_TRQ_LIVE:  // open-loop, torque steps live
        torque = tsTrqLive();
        input.refAcc_ = 0;
        input.refVel_ = 0;
        break;

    case MODE_CL_VEL_LIVE:  // closed-loop, accel. steps according to speed param
        tsVelLive(v_veh, input.refAcc_, input.refVel_);
        break;

    default: // closed-loop, external reference (Autoware)
        ;// nothing to do here
    }

    // feedback
    // TODO:  consider external torque limits
    if (active_ctrl_mode_ >= MODE_CL_VEL_LIVE)
        torque = controller0(input.refAcc_, input.refVel_, v_veh, input.curSlope_, p_trq_max_, p_trq_min_);

#ifdef LON_CTRL_DBG_OUT
    dbg_.ref_acc = input.refAcc_;
    dbg_.ref_vel = input.refVel_;
    dbg_.trq_total = torque;
    dbg_.v_gpt = input.curVel_;
    dbg_.v_rpm = input.curVel_;
    dbg_pub_->publish(dbg_);
#endif
    return torque;
}

double vifware_vehicle_interfaces::LongitudinalControl::controller0(const double a_ref_k, const double v_ref_k, const double v_meas_k, const double alpha, const double trq_max, const double trq_min)
{
    // constants
    constexpr double g_acc = 9.81;
    constexpr double rho_air = 1.2;

    // reference jerk
    static double a_ref_km1 = 0;
    const double j_k = (a_ref_k - a_ref_km1) / p_ctrl_rate_s_;
    a_ref_km1 = a_ref_k;

    // feedforward terms
    const double F_accel = p_veh_mass_ * a_ref_k;
    const double F_air = 0.5 * p_veh_front_A_ * p_veh_cw_ * rho_air * v_ref_k * v_ref_k * sign(v_ref_k);
    const double F_roll = p_veh_cr_ * p_veh_mass_ * g_acc * cos(alpha) * sign(v_ref_k);
    const double F_slope = p_veh_mass_ * g_acc * sin(alpha);
    const double T_em = (F_accel + F_air + F_roll + F_slope) * p_veh_r_wheel_ / p_veh_i_fd_;
    const double T_em_dot = (p_veh_r_wheel_ / p_veh_i_fd_) * (p_veh_mass_ * j_k + p_veh_front_A_ * p_veh_cw_ * rho_air * v_ref_k * a_ref_k);
    const double T_em_ff = T_em + p_ctrl0_tc_em_ff_ * T_em_dot;

    // feedback terms
    const double e_k = v_ref_k - v_meas_k;
    const double a_k = p_ctrl0_ka_ * p_ctrl_rate_s_ * (u_sat_km1_ - u_km1_);
    const double e_tilde_k = e_k + a_k;
    const double e_tilde_km1 = e_km1_ + a_km1_;
    // linear integrator
    const double u_lint_k = fmax(-p_ctrl0_lint_lim_, fmin(p_ctrl0_lint_lim_, u_lint_km1_ + 0.5 * p_ctrl0_ki_l_ * p_ctrl_rate_s_ * (e_tilde_k + e_tilde_km1)));
    // nonlinear integrator
    const double u_nlint_k = fmax(-p_ctrl0_nlint_lim_, fmin(p_ctrl0_nlint_lim_, u_nlint_km1_ + 0.5 * p_ctrl0_ki_nl_ * p_ctrl_rate_s_ * (sign(e_tilde_k) + sign(e_tilde_km1))));

    const double u_fb_k = p_ctrl0_kp_ * e_k + u_lint_k + u_nlint_k + p_ctrl0_kd_ * (e_k - e_km1_) / p_ctrl_rate_s_;
    const double u_k = u_fb_k + p_ctrl0_w_ff_ * T_em_ff;      // desired output: feedback + weighted (0 - 1) feedforward
    const double u_sat_k = fmax(trq_min, fmin(trq_max, u_k)); // real output: saturation

    // store values for next call
    a_km1_ = a_k;
    e_km1_ = e_k;
    u_km1_ = u_k;
    u_lint_km1_ = u_lint_k;
    u_nlint_km1_ = u_nlint_k;
    u_sat_km1_ = u_sat_k;

#ifdef LON_CTRL_DBG_OUT
    dbg_.err_vel = e_k;
    dbg_.trq_fb_int = u_lint_k + u_nlint_k;
    dbg_.trq_fb = u_fb_k;
#endif
    return u_sat_k;
}

double vifware_vehicle_interfaces::LongitudinalControl::tsTrqLive()
{
    if (!ts_phase_) // initial delay
    {
        const auto timeDiff = (node_->now() - ts_phase_start_).seconds();

        if (timeDiff >= p_ts_t_delay_)
        {
            ts_phase_++;
            ts_phase_start_ += std::chrono::duration<double>(p_ts_t_delay_);
        }

        return 0;
    }

    return fmax(p_trq_min_, fmin(p_trq_max_, p_ts_trq_));
}

void vifware_vehicle_interfaces::LongitudinalControl::tsVelLive(const double vCur, double &refAcc, double &refVel)
{
    static double refVelOld = 0;
    static double tsVelOld = 0;
    static double T_a = 0;
    static double a = 0;

    if (!ts_phase_) // initial delay
    {
        const auto timeDiff = (node_->now() - ts_phase_start_).seconds();

        if (timeDiff >= p_ts_t_delay_)
        {
            ts_phase_++;
            ts_phase_start_ += std::chrono::duration<double>(p_ts_t_delay_);
        }

        refVelOld = 0;
        tsVelOld = 0;
        refAcc = 0;
        refVel = 0;
        a = 0;
        T_a = 0;
        return;
    }

    if (fabs(p_ts_v_ - tsVelOld) > 1e-1)
    {
        ts_phase_start_ = node_->now();
        refVelOld = vCur;
        tsVelOld = p_ts_v_;
        a = (p_ts_v_ >= vCur) ? p_ts_a_acc_ : p_ts_a_dec_;
        T_a = (p_ts_v_ - vCur) / a;
    }

    const auto timeDiff = (node_->now() - ts_phase_start_).seconds();

    if (timeDiff >= T_a)
    {
        refAcc = 0;
        refVel = p_ts_v_;
    }
    else
    {
        refAcc = a;
        refVel = refVelOld + timeDiff * a;
    }
}