#include "bno086_cli_service.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

void on_cli_request(const std::shared_ptr<bno086_cli_interface::srv::Bno086Cli::Request> request,
                    std::shared_ptr<bno086_cli_interface::srv::Bno086Cli::Response>      response)
{
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "bno086_cli_service::on_cli_request(\"%s\")", request->cli_cmd_text.c_str());
}

void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,       // CHANGE
         std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>      response)           // CHANGE
{
    response->sum = request->a + request->b + request->c;       // CHANGE
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Incoming request\na: %ld"
                " b: %ld"
                " c: %ld",                                 // CHANGE
                request->a, request->b, request->c);       // CHANGE
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}