// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roboclaw_hardware_interface/roboclaw_hardware_interface.hpp"

#include <iostream>
#include <roboclaw_serial/device.hpp>

#include "rclcpp/rclcpp.hpp"

namespace roboclaw_hardware_interface
{
    CallbackReturn RoboClawHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        const HardwareInfo &hardware_info = params.hardware_info;
    // }
    // //-------------------------------------------------------------------------------------
    // CallbackReturn RoboClawHardwareInterface::on_init(const HardwareInfo &hardware_info)
    // {

        CallbackReturn ret = hardware_interface::SystemInterface::on_init(params);
        if (ret != CallbackReturn::SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawHardwareInterface"),
                             "RoboClawHardwareInterface::on_init() SystemInterface::on_init(hardware_info) failed");
                return ret;
            }

        // Validate serial port parameter
        std::string serial_port;
        try
            {
                serial_port = hardware_info.hardware_parameters.at("serial_port");
            }
        catch (const std::out_of_range &)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawHardwareInterface"),
                             "RoboClawHardwareInterface::on_init() 'serial_port' must be defined as a hardware parameter.");
                return CallbackReturn::ERROR;
            }

        // create roboclaw classes
        try
            {
                // Read the serial port from hardware parameters
                auto device = std::make_shared<roboclaw_serial::SerialDevice>(serial_port);
                interface_  = std::make_shared<roboclaw_serial::Interface>(device);
            }
        catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawHardwareInterface::on_init() create device, exception--> %s", e.what());
                return CallbackReturn::FAILURE;
            }

        // Validate parameters describing roboclaw joint configurations
        RoboClawConfiguration config;
        try
            {
                config = parse_roboclaw_configuration(hardware_info);
            }
        catch (const std::runtime_error &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawHardwareInterface"),
                             "RoboClawHardwareInterface::on_init() parse_roboclaw_configuration, exception--> %s", e.what());
                return CallbackReturn::ERROR;
            }
        roboclaw_config = config;

        for (auto &[roboclaw_address, joints] : config)
            {
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "Claw_addr = %d", roboclaw_address);
                for (auto &[joint_name, joint] : joints)
                    {
                        if (joint)
                            {
                                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "    Joint = %s    or %s", joint_name.c_str(), joint->name.c_str());
                            }
                    }
            }

        // Initialize each roboclaw unit from validated configuration
        for (auto &[roboclaw_address, joints] : config) { roboclaw_units_.push_back(RoboClawUnit(interface_, roboclaw_address, joints["M1"], joints["M2"])); }

        try
            {
                for (auto &roboclaw : roboclaw_units_) { roboclaw.read_firmware_version(); }
            }
        catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawHardwareInterface"),
                             "RoboClawHardwareInterface::on_init() roboclaw.read_firmware_version(), exception--> %s", e.what());
                return CallbackReturn::ERROR;
            }

        return CallbackReturn::SUCCESS;
    }

    //-------------------------------------------------------------------------------------
    // std::vector<StateInterface> RoboClawHardwareInterface::export_state_interfaces()
    // {
    //     std::vector<StateInterface> state_interfaces;
    //     for (auto roboclaw : roboclaw_units_)
    //         {
    //             for (auto &joint : roboclaw.joints)
    //                 {
    //                     if (joint) { state_interfaces.emplace_back(joint->name, "position", joint->getPositionStatePtr()); }
    //                 }
    //         }
    //     RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "export_state_interfaces() count=%zu", state_interfaces.size());
    //     return state_interfaces;
    // }

    std::vector<StateInterface::ConstSharedPtr> RoboClawHardwareInterface::on_export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "============= on_export_state_interfaces() =========================");
        auto zzz = SystemInterface::on_export_state_interfaces();

        for (auto unit : roboclaw_units_)
            {
                for (auto &joint : unit.joints)
                    {
                        if (joint)
                            {
                                std::string joint_name     = joint->name;
                                std::string interface_name = joint_name + "/position";
                                for (const auto &state_interface : joint_states_)
                                    {
                                        if (state_interface->get_name() == interface_name)
                                            {
                                                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"),
                                                            "on_export_state_interfaces() interface_name=%s found.", interface_name.c_str());
                                                joint->set_position_state_interface(state_interface);
                                            }
                                    }
                            }
                    }
            }

        // DumpConfig();
        return zzz;
    }

    //-------------------------------------------------------------------------------------

    std::vector<CommandInterface::SharedPtr> RoboClawHardwareInterface::on_export_command_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "============= on_export_command_interfaces() =========================");
        auto zzz = SystemInterface::on_export_command_interfaces();

        for (auto unit : roboclaw_units_)
            {
                for (auto &joint : unit.joints)
                    {
                        if (joint)
                            {
                                std::string joint_name     = joint->name;
                                std::string interface_name = joint_name + "/velocity";
                                for (const auto &cmd_interface : joint_commands_)
                                    {
                                        if (cmd_interface->get_name() == interface_name)
                                            {
                                                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"),
                                                            "on_export_command_interfaces() interface_name=%s found.", interface_name.c_str());
                                                joint->set_velocity_command_interface(cmd_interface);
                                            }
                                    }
                            }
                    }
            }

        // DumpConfig();
        return zzz;
    }

    // std::vector<StateInterface::ConstSharedPtr> RoboClawHardwareInterface::on_export_state_interfaces()
    // {
    //     std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    //     struct InterfaceInfo if_info;
    //     if_info.name = "position";
    //     if_info.type = "double";
    //     struct InterfaceDescription if_desc("", if_info);
    //     for (auto roboclaw : roboclaw_units_)
    //         {
    //             for (auto &joint : roboclaw.joints)
    //                 {
    //                     if (joint) {
    //                         if_desc.name = joint->name;
    //                         auto state_ptr = std::make_shared<hardware_interface::StateInterface> (if_desc);
    //                         state_interfaces_[joint->name] = state_ptr;
    //                         state_interfaces.emplace_back(state_ptr);
    //                         }
    //                 }
    //         }
    //     RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "on_export_state_interfaces() count=%zu", state_interfaces.size());
    //     return state_interfaces;
    // }

    //-------------------------------------------------------------------------------------
    // std::vector<CommandInterface> RoboClawHardwareInterface::export_command_interfaces()
    // {
    //     std::vector<CommandInterface> command_interfaces;
    //     for (auto roboclaw : roboclaw_units_)
    //         {
    //             for (auto &joint : roboclaw.joints)
    //                 {
    //                     if (joint) { command_interfaces.emplace_back(joint->name, "velocity", joint->getVelocityCommandPtr()); }
    //                 }
    //         }
    //     RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "export_state_interfaces() count=%zu", command_interfaces.size());
    //     return command_interfaces;
    // }

    //-------------------------------------------------------------------------------------
    return_type RoboClawHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (auto &roboclaw : roboclaw_units_) { roboclaw.write(); }
        return return_type::OK;
    }

    //-------------------------------------------------------------------------------------
    return_type RoboClawHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (auto &roboclaw : roboclaw_units_) { roboclaw.read(); }
        return return_type::OK;
    }

    //-------------------------------------------------------------------------------------
    RoboClawConfiguration RoboClawHardwareInterface::parse_roboclaw_configuration(const HardwareInfo &hardware_info)
    {
        // Define the configuration map
        RoboClawConfiguration claw_config;

        // Loop over all motors and associate them with their respective roboclaws
        for (auto joint : hardware_info.joints)
            {
                // We currently only support velocity command interfaces
                if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "velocity")
                    {
                        throw std::runtime_error("Invalid command interface for " + joint.name + ". Only velocity command interfaces are supported.");
                    }

                // We currently only support position state interfaces
                if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "position")
                    {
                        throw std::runtime_error("Invalid state interface for " + joint.name + ". Only position state interfaces are supported.");
                    }

                // Capture and validate parameters
                uint8_t roboclaw_address;
                try
                    {
                        roboclaw_address = static_cast<uint8_t>(stoi(joint.parameters.at("address")));

                        if (roboclaw_address < 0x80 || roboclaw_address > 0x87)
                            {
                                throw std::runtime_error(joint.name + ": Addresses must be in the range [128:136]");
                            }
                    }
                catch (const std::invalid_argument &e)
                    {
                        std::cerr << e.what() << std::endl;
                        throw std::runtime_error(joint.name + ": Address must be an integer.");
                    }
                catch (const std::exception &e)
                    {
                        std::cerr << e.what() << std::endl;
                        throw std::runtime_error("Problem looking up address and converting to uint8_t");
                    }

                // Get the tick count per wheel rotation value
                int qppr;
                try
                    {
                        qppr = stoi(joint.parameters.at("qppr"));
                    }
                catch (const std::out_of_range &)
                    {
                        throw std::runtime_error("qppr is not set for " + joint.name);
                    }
                catch (const std::invalid_argument &)
                    {
                        throw std::runtime_error("qppr is not numeric for " + joint.name);
                    }

                // Get the wheel diameter value
                double wheel_diameter_meters = 0;
                try
                    {
                        wheel_diameter_meters = stod(joint.parameters.at("wheel_diameter"));
                    }
                catch (const std::out_of_range &)
                    {
                        throw std::runtime_error("wheel_diameter is not set for " + joint.name);
                    }
                catch (const std::invalid_argument &)
                    {
                        throw std::runtime_error("wheel_diameter is not numeric for " + joint.name);
                    }

                // Get the type of motor from joint parameters
                std::string motor_type;
                try
                    {
                        motor_type = joint.parameters.at("motor_type");
                    }
                catch (const std::out_of_range &)
                    {
                        throw std::runtime_error("Motor type not set for " + joint.name + ". It must be either M1 or M2.");
                    }

                // Ensure that the motor type is valid
                if (motor_type != "M1" && motor_type != "M2")
                    {
                        throw std::runtime_error("Motor type for " + joint.name + " must be either M1 or M2 (" + motor_type + " provided).");
                    }

                // Ensure that a key exists for this address, otherwise initialize default values
                claw_config.emplace(roboclaw_address, std::map<std::string, MotorJoint::SharedPtr>({{"M1", nullptr}, {"M2", nullptr}}));

                // Ensure that this motor has not already been configured
                if (!claw_config[roboclaw_address][motor_type])
                    {
                        // Set configuration parameters for this motor
                        auto pJoint                               = std::make_shared<MotorJoint>(joint.name, qppr, wheel_diameter_meters);
                        claw_config[roboclaw_address][motor_type] = pJoint;
                    }
                else { throw std::runtime_error("Bad motor type " + motor_type + " specified for joint " + joint.name); }
            }

        return claw_config;
    }       //--- End of parse_roboclaw_configuration

    void RoboClawHardwareInterface::DumpConfig()
    {
        RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "************************** DumpConfig **************************");
        // std::unordered_map<std::string, InterfaceDescription> joint_state_interfaces_
        for (const auto &[key, descr] : joint_state_interfaces_)
            {
                std::string prefix_name               = descr.get_prefix_name();
                std::string name                      = descr.get_name();
                std::string interface_info__name      = descr.interface_info.name;
                std::string interface_info__data_type = descr.interface_info.data_type;
                std::string interface_info__min       = descr.interface_info.min;
                std::string interface_info__max       = descr.interface_info.max;
                std::string params;
                for (const auto &[key2, value] : descr.interface_info.parameters) { params += key2 + "='" + value + "'  "; }
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"),
                            "joint_state_interfaces_ key='%s'  prefix_name=%s name=%s  interface_info:[name=%s data_type=%s min=%s max=%s  [params=%s]]",
                            key.c_str(), prefix_name.c_str(), name.c_str(), interface_info__name.c_str(), interface_info__data_type.c_str(),
                            interface_info__min.c_str(), interface_info__max.c_str(), params.c_str());
            }

        // std::unordered_map<std::string, InterfaceDescription> joint_command_interfaces_
        for (const auto &[key, descr] : joint_command_interfaces_)
            {
                std::string prefix_name               = descr.get_prefix_name();
                std::string name                      = descr.get_name();
                std::string interface_info__name      = descr.interface_info.name;
                std::string interface_info__data_type = descr.interface_info.data_type;
                std::string interface_info__min       = descr.interface_info.min;
                std::string interface_info__max       = descr.interface_info.max;
                std::string params;
                for (const auto &[key2, value] : descr.interface_info.parameters) { params += key2 + "='" + value + "'  "; }
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"),
                            "joint_command_interfaces_ key='%s' prefix_name=%s name=%s  interface_info:[name=%s data_type=%s min=%s max=%s  [params=%s]]",
                            key.c_str(), prefix_name.c_str(), name.c_str(), interface_info__name.c_str(), interface_info__data_type.c_str(),
                            interface_info__min.c_str(), interface_info__max.c_str(), params.c_str());
            }

        // std::vector<StateInterface::SharedPtr> joint_states_;
        for (const auto &state_interface : joint_states_)
            {
                std::string prefix_name    = state_interface->get_prefix_name();
                std::string name           = state_interface->get_name();
                std::string interface_name = state_interface->get_interface_name();
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "joint_states_  prefix=%s name=%s  interface_name=%s", prefix_name.c_str(),
                            name.c_str(), interface_name.c_str());
            }

        // std::vector<CommandInterface::SharedPtr> joint_commands_;
        for (const auto &command_interface : joint_commands_)
            {
                std::string prefix_name    = command_interface->get_prefix_name();
                std::string name           = command_interface->get_name();
                std::string interface_name = command_interface->get_interface_name();
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "joint_commands_  prefix=%s name=%s  interface_name=%s", prefix_name.c_str(),
                            name.c_str(), interface_name.c_str());
            }
        RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "******************** end of DumpConfig **************************");
    }

}       // namespace roboclaw_hardware_interface
        //-------------------------------------------------------------------------------------

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(roboclaw_hardware_interface::RoboClawHardwareInterface, hardware_interface::SystemInterface);
