#pragma once

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>

namespace vifware_vehicle_interfaces {

    using AutowareChangeModeSrv = autoware_adapi_v1_msgs::srv::ChangeOperationMode;

    using ServiceResponseFuture =
        rclcpp::Client<AutowareChangeModeSrv>::SharedFutureWithRequest;

    class AutowareAdApiClient
    {
    public:

        explicit AutowareAdApiClient(const std::shared_ptr<rclcpp::Node> &node);

        void autonomousModeRequest();
        void stopModeRequest();

    private:

        void initComm();

        void autonomousModeResponseCallback(ServiceResponseFuture future);
        void stopModeResponseCallback(ServiceResponseFuture future);

        rclcpp::Node::SharedPtr node_;

        bool enable_;

        rclcpp::Client<AutowareChangeModeSrv>::SharedPtr to_autonomous_mode_client_;
        rclcpp::Client<AutowareChangeModeSrv>::SharedPtr to_stop_mode_client_;

    };

} // namespace vifware_vehicle_interfaces
