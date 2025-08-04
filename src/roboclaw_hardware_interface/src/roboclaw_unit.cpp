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

#include "roboclaw_hardware_interface/roboclaw_unit.hpp"

#include <map>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

    RoboClawUnit::RoboClawUnit(roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1, MotorJoint::SharedPtr m2)
        : address_(address)
    {
        // Copy the pointer to the roboclaw interface
        interface_ = interface;

        // Set motor joint configurations
        joints[0] = m1;
        joints[1] = m2;

        // // auto log = rclcpp::get_logger("RoboclawHardwareInterface");

        // // int sz_tick_rate = getReadSize(tick_rate_command_.fields);
        // // RCLCPP_INFO(log, "RoboClawUnit::RoboClawUnit()  sz_tick_rate = %d", sz_tick_rate);
        // // int sz_encoder_state = getReadSize(encoder_state_.fields);
        // // RCLCPP_INFO(log, "RoboClawUnit::RoboClawUnit()  sz_encoder_state = %d", sz_encoder_state);
        // // int sz_main_battery_voltage = getReadSize(main_battery_voltage_.fields);
        // // RCLCPP_INFO(log, "RoboClawUnit::RoboClawUnit()  sz_main_battery_voltage = %d", sz_main_battery_voltage);
        // // int sz_firmware_version = getReadSize(firmware_version_.fields);
        // // RCLCPP_INFO(log, "RoboClawUnit::RoboClawUnit()  sz_firmware_version = %d", sz_firmware_version);
    }

    // Read the encoder counts from the roboclaw and update position state
    void RoboClawUnit::read()
    {
        // Read and update position
        interface_->read(encoder_state_, address_);
        read_count_++;

        // Get constant references to fields in the encoder counts message
        const auto &[m1_ticks, m2_ticks] = encoder_state_.fields;
        int32_t m1_ticks_nc = m1_ticks;
        int32_t m2_ticks_nc = m2_ticks;

        // Convert tick counts to position states for each field if the corresponding joint exists
        if (joints[0])
            {
                if (m1_ticks_nc == 0) { m1_ticks_nc = joints[0]->getProjectedEncoderCount(); }
                joints[0]->setPositionState(m1_ticks_nc);
            }
        if (joints[1]) {  
            if (m2_ticks_nc == 0) { m2_ticks_nc = joints[1]->getProjectedEncoderCount(); }
            joints[1]->setPositionState(m2_ticks_nc);
         }

        // Read the main battery voltage
        if (elap_since_last_MainBatteryVoltage > elap_since_last_MainBatteryVoltage_span)
            {
                elap_since_last_MainBatteryVoltage = 0;
                interface_->read(main_battery_voltage_, address_);
                const auto &[voltage] = main_battery_voltage_.fields;
                double voltage_F      = voltage / 10.0;
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawUnit::read()  Main Battery Voltage = %.1f", voltage_F);
                main_battery_voltage_in_10ths_of_volts = voltage;
            }
            
        // dump distributions every 2 minutes
        if (elap_since_last_dump_distributions > elap_since_last_dump_distributions_span)
            {
                elap_since_last_dump_distributions = 0;
                interface_->dump_distributions();
            }
    }

    // Write the tick rate request to the roboclaw and update
    void RoboClawUnit::write()
    {
        // Get references to fields in the tick rate command message
        auto &[m1_speed, m2_speed] = tick_rate_command_.fields;

        // Set values to each field if the corresponding joint exists
        if (joints[0]) { m1_speed = joints[0]->getTickRateCommand(); }
        if (joints[1]) { m2_speed = joints[1]->getTickRateCommand(); }

        // Write the rate request to the roboclaw driver
        interface_->write(tick_rate_command_, address_);

        auto [m1_actual_speed, m2_actual_speed] = tick_rate_command_.fields;
        if (m1_actual_speed != m1_actual_speed_in_ticks_previous || m2_actual_speed != m2_actual_speed_in_ticks_previous)
            {
                m1_actual_speed_in_ticks_previous = m1_actual_speed;
                m2_actual_speed_in_ticks_previous = m2_actual_speed;
                write_count_++;
                // RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawUnit::write()  m[1|2]_actual_speed = %5d / %5d ", m1_actual_speed, m2_actual_speed);
            }
    }

    void RoboClawUnit::read_firmware_version()
    {

        //int tick_size = tick_rate_command_.getReadSize();


        // Read the firmware version from the roboclaw
        interface_->read(firmware_version_, address_);
        const auto &[version] = firmware_version_.fields;
        std::string version_str(version.begin(), version.end());
        int         sz = version_str.size();
        for (int i = 0; i < sz; i++)
            {
                if (version_str[i] == '\n' || version_str[i] == '\r') { version_str[i] = ' '; }
            }

        RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawUnit::read_firmware_version() = %s", version_str.c_str());
    }

}       // namespace roboclaw_hardware_interface
