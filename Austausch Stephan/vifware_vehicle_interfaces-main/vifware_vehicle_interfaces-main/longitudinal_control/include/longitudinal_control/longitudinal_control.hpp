// Copyright VIF

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vifware_vehicle_interface_msgs/msg/longitudinal_control_debug.hpp>

// enable debug signals
#define LON_CTRL_DBG_OUT

namespace vifware_vehicle_interfaces {

    struct LonCtrlInput
    {
        // desired acceleration and speed
        double refAcc_ = 0;
        double refVel_ = 0;
        // currently measured values
        double curVel_ = 0;
        double curSlope_ = 0;
    };

    class LongitudinalControl
    {
    public:
        explicit LongitudinalControl(const std::shared_ptr<rclcpp::Node> &node, const double ctrlRate);

        using LonCtrlDbg = vifware_vehicle_interface_msgs::msg::LongitudinalControlDebug;

        static constexpr const char *PARAM_NS = "lon_ctrl.";
        static constexpr int64_t MODE_IDLE = 0;
        static constexpr int64_t MODE_OL_TRQ_LIVE = 1;
        static constexpr int64_t MODE_CL_VEL_LIVE = 2;
        static constexpr int64_t MODE_CL_EXT_REF = 3;

        double calc(LonCtrlInput &input);
        void disable() { active_ctrl_mode_ = MODE_IDLE; }
        void enable() { active_ctrl_mode_ = p_ctrl_mode_; }
        bool isEnabled() { return active_ctrl_mode_ >= MODE_CL_VEL_LIVE; }

        // vehicle parameters
        double p_veh_cr_;      // rolling resistance coeff.
        double p_veh_cw_;      // air resistance coeff.
        double p_veh_front_A_; // vehicle's front area
        double p_veh_i_fd_;    // final drive gear ratio
        double p_veh_mass_;    // vehicle mass
        double p_veh_r_wheel_; // wheel radius

    private:
        // callbacks
        rcl_interfaces::msg::SetParametersResult cbParams(const std::vector<rclcpp::Parameter> &parameters);

        // controller variants
        double controller0(const double a_ref_k, const double v_ref_k, const double v_meas_k, const double alpha, const double trq_max, const double trq_min);

        // test signal generators
        double tsTrqLive();
        void tsVelLive(const double vCur, double &refAcc, double &refVel);

        // implement component
        rclcpp::Node::SharedPtr node_;

        // controller parameters
        int64_t p_ctrl_mode_;  // desired control mode when enabled
        double p_ctrl_rate_s_;

        // controller version 0
        double p_ctrl0_lint_lim_;
        double p_ctrl0_nlint_lim_;
        double p_ctrl0_ka_;
        double p_ctrl0_kp_;
        double p_ctrl0_ki_l_;
        double p_ctrl0_ki_nl_;
        double p_ctrl0_kd_;
        double p_ctrl0_w_ff_;
        double p_ctrl0_tc_em_ff_;

        // test signal
        double p_ts_a_acc_;
        double p_ts_a_dec_;
        double p_ts_v_;
        double p_ts_trq_;
        double p_ts_t_delay_;

        // torque limits
        double p_trq_max_;
        double p_trq_min_;

        // controller working set
        int64_t active_ctrl_mode_ = MODE_IDLE;
        double a_km1_ = 0;
        double e_km1_ = 0;
        double u_km1_ = 0;
        double u_lint_km1_ = 0;
        double u_nlint_km1_ = 0;
        double u_sat_km1_ = 0;

        // test signals
        uint32_t ts_phase_;
        rclcpp::Time ts_phase_start_;

        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr dynParamHandle_;

#ifdef LON_CTRL_DBG_OUT
        // debug messages for controller calibration
        LonCtrlDbg dbg_;
        rclcpp::Publisher<LonCtrlDbg>::SharedPtr dbg_pub_;
#endif
    };

} // namespace vifware_vehicle_interfaces