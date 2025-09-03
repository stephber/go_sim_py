#include <autoware_interface/autoware_ad_api_client.hpp>

namespace vifware_vehicle_interfaces
{
    using namespace std::chrono_literals;

    AutowareAdApiClient::AutowareAdApiClient(const std::shared_ptr<rclcpp::Node> &node)
        : node_(node)
    {
        enable_ = node_->declare_parameter("enable_autoware_ad_api_client", false);
        if (enable_) {
            initComm();
        }
    }

    void AutowareAdApiClient::initComm()
    {
        to_autonomous_mode_client_ =
            this->node_->create_client<AutowareChangeModeSrv>("/api/operation_mode/change_to_autonomous");

        to_stop_mode_client_ =
            this->node_->create_client<AutowareChangeModeSrv>("/api/operation_mode/change_to_stop");
    }

    void AutowareAdApiClient::autonomousModeRequest()
    {
        if (!enable_) return;

        auto logger = this->node_->get_logger();
        RCLCPP_INFO(logger, "vehicle interface AdApi client: to_autonomous service call");

        if (!to_autonomous_mode_client_->service_is_ready()) {
            RCLCPP_ERROR(logger, "vehicle interface AdApi client: to_autonomous service NOT ready");
            return;
        }

        auto request = std::make_shared<AutowareChangeModeSrv::Request>();
        auto result = to_autonomous_mode_client_->async_send_request
            (request,
             // callback when response is received
             [this](ServiceResponseFuture future) {
                 this->autonomousModeResponseCallback(future);
             });
        RCLCPP_INFO(logger,
                    "Send request to service '/api/operation_mode/change_to_autonomous'");
    }

    void AutowareAdApiClient::autonomousModeResponseCallback(ServiceResponseFuture future)
    {
        auto logger = this->node_->get_logger();
        auto request_response_pair = future.get();
        if (request_response_pair.second->status.success) {
            RCLCPP_INFO(logger,
                        "Sucessfully called service '/api/operation_mode/change_to_autonomous");
        } else {
            RCLCPP_ERROR(logger,
                         "Failed to call called service '/api/operation_mode/change_to_autonomous");
        }
    }

    void AutowareAdApiClient::stopModeRequest()
    {
        if (!enable_) return;
        auto logger = this->node_->get_logger();
        RCLCPP_INFO(logger, "vehicle interface AdApi client: to_stop service call");

        if (!to_stop_mode_client_->service_is_ready()) {
            RCLCPP_ERROR(logger, "vehicle interface AdApi client: to_stop service NOT ready");
            return;
        }

        auto request = std::make_shared<AutowareChangeModeSrv::Request>();
        auto result = to_stop_mode_client_->async_send_request
            (request,
             // callback when response is received
             [this](ServiceResponseFuture future) {
                 this->stopModeResponseCallback(future);
             });
        RCLCPP_INFO(logger, "Send request to service '/api/operation_mode/change_to_stop'");
    }

    void AutowareAdApiClient::stopModeResponseCallback(ServiceResponseFuture future)
    {
        auto logger = this->node_->get_logger();
        auto request_response_pair = future.get();
        if (request_response_pair.second->status.success) {
            RCLCPP_INFO(logger,
                        "Sucessfully called service '/api/operation_mode/change_to_stop");
        } else {
            RCLCPP_ERROR(logger,
                         "Failed to call called service '/api/operation_mode/change_to_stop");
        }
    }

}  // namespace
