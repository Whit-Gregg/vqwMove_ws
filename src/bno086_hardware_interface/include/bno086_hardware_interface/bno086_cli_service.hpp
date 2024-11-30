#pragma once

#include "Bno086Hardwareinterface.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/string.hpp"

namespace bno086_hardware_interface
{

    class bno086_cli_service
    {
      public:
        bno086_cli_service(Bno086Hardwareinterface *pHW) : pParent(pHW) {}

        ~bno086_cli_service()
        {
            
        }

        using request_t  = std::shared_ptr<bno086_cli_interface::srv::Bno086Cli::Request>;
        using response_t = std::shared_ptr<bno086_cli_interface::srv::Bno086Cli::Response>

            void on_cli_request(const request_t request, response_t response);

      private:
        void init()
        {
            pNode         = rclcpp::Node::make_shared("bno086_cli_service_node");
            auto callback = [this](const request_t request, response_t response) { this->on_cli_request(request, response); }

            pService = pNode->create_service<bno086_cli_interface::srv::Bno086Cli>("bno086_cli", callback);
        }

        Bno086Hardwareinterface                                         *pParent;
        std::shared_ptr<rclcpp::Node>                                    pNode;
        rclcpp::Service<bno086_cli_interface::srv::Bno086Cli>::SharedPtr pService;

    };       // end of:  class bno086_cli_service

}       // namespace bno086_hardware_interface