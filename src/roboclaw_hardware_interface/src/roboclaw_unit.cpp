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
        elapsedMillis elap_read;

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

        // Display the tick counts every 200 ms
        if (elap_since_ticks_display > elap_since_ticks_display_span) {
                int32_t elap = elap_since_ticks_display;
            elap_since_ticks_display = 0;
         if (m1_ticks_nc!=0 && m2_ticks_nc!=0){
            if (m1_ticks_nc != m1_ticks_nc_previous || m2_ticks_nc != m2_ticks_nc_previous)
            {
                elap_since_ticks_display = 0;
                int32_t m1_ticks_diff = m1_ticks_nc - m1_ticks_nc_previous;
                int32_t m2_ticks_diff = m2_ticks_nc - m2_ticks_nc_previous;
                double elap_sec = elap / 1000.0;
                if (elap_sec == 0) { elap_sec = 0.001; }
                double m1_tickRate = m1_ticks_diff / elap_sec;
                double m2_tickRate = m2_ticks_diff / elap_sec;
                auto log = rclcpp::get_logger("RoboclawHardwareInterface");
                RCLCPP_INFO(log, "RoboClawUnit::read()  m1_ticks=%d, m2_ticks=%d, m1_ticks_diff=%d, m2_ticks_diff=%d, m1_tickRate=%.1f, m2_tickRate=%.1f, elap=%.3f seconds",
                            m1_ticks_nc, m2_ticks_nc, m1_ticks_diff, m2_ticks_diff, m1_tickRate, m2_tickRate, elap_sec);
                m1_ticks_nc_previous = m1_ticks_nc;
                m2_ticks_nc_previous = m2_ticks_nc;
            }
        }
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

        if (elap_since_last_Roboclaw_Status > elap_since_last_Roboclaw_Status_span)
            {
                elap_since_last_Roboclaw_Status = 0;
                
                interface_->read(roboclaw_status_, address_);
                const auto &[status] = roboclaw_status_.fields;
                if (( status != roboclaw_status_previous )|| (roboclaw_status_display_count % 10 == 0)) {
                    std::string status_str = roboclaw_status_as_string(status);
                    RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawUnit::read()  Roboclaw Status = 0x%X (%s)", status, status_str.c_str());
                }
                roboclaw_status_previous = status;
                roboclaw_status_display_count++;
            }

        // dump distributions every 2 minutes
        if (elap_since_last_dump_distributions > elap_since_last_dump_distributions_span)
            {
                elap_since_last_dump_distributions = 0;
                interface_->dump_distributions();
                dump_distributions();
            }

        // Read the instantaneous speeds
        if (elap_since_InstantaneousSpeeds > elap_since_InstantaneousSpeeds_span)
            {
                elap_since_InstantaneousSpeeds = 0;
                interface_->read(instantaneous_speeds_, address_);
                const auto &[m1_speed, m2_speed] = instantaneous_speeds_.fields;
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawUnit::read()  Instantaneous Speeds: M1 = %d, M2 = %d", m1_speed, m2_speed);
            }

            read_time_distribution.addValue(elap_read);
        }

    // Write the tick rate request to the roboclaw and update
    void RoboClawUnit::write()
    {
        elapsedMillis elap_write;

        // Get references to fields in the tick rate command message
        auto &[m1_speed, m2_speed] = tick_rate_command_.fields;

        // auto &m1_speed_m1 = std::get<0>(m1_tick_rate_command_.fields);
        // auto &m2_speed_m2 = std::get<0>(m2_tick_rate_command_.fields);

        int32_t m1_speed_target = 0;
        int32_t m2_speed_target = 0;

        // Set values to each field if the corresponding joint exists
        if (joints[0]) { m1_speed_target = joints[0]->getTickRateCommand(); }
        if (joints[1]) { m2_speed_target = joints[1]->getTickRateCommand(); }

        m1_speed = m1_speed_target;
        m2_speed = m2_speed_target;

        // m1_speed_m1 = m1_speed_target;
        // m2_speed_m2 = m2_speed_target;

        // // Write the rate request to the roboclaw driver

        interface_->write(tick_rate_command_, address_);
        // interface_->write(m1_tick_rate_command_, address_);
        // interface_->write(m2_tick_rate_command_, address_);

        auto [m1_actual_speed, m2_actual_speed] = tick_rate_command_.fields;
        // auto m1_actual_speed = std::get<0>(m1_tick_rate_command_.fields);
        // auto m2_actual_speed = std::get<0>(m2_tick_rate_command_.fields);

        if (m1_actual_speed != m1_actual_speed_in_ticks_previous || m2_actual_speed != m2_actual_speed_in_ticks_previous)
            {
                m1_actual_speed_in_ticks_previous = m1_actual_speed;
                m2_actual_speed_in_ticks_previous = m2_actual_speed;
                write_count_++;
                RCLCPP_INFO(rclcpp::get_logger("RoboclawHardwareInterface"), "RoboClawUnit::write()  m[1|2]_actual_speed = %5d / %5d ", m1_actual_speed, m2_actual_speed);
            }

            write_time_distribution.addValue(elap_write);
    }

    void RoboClawUnit::read_firmware_version()
    {
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

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        void RoboClawUnit::dump_distributions()
        {
            auto log = rclcpp::get_logger("RoboclawHardwareInterface");
            RCLCPP_INFO(log, "RoboClawUnit::dump_distributions()-------------");

            // read_time_distribution
            RCLCPP_INFO(log, "RoboClawUnit::dump_distributions()---  read_time_distribution  ----------");
            for (int x = 0; x < read_time_distribution.MAX_VALUE; x++)
                {
                    long count = read_time_distribution.getValue(x);
                    if (count > 0) { RCLCPP_INFO(log, "RoboClawUnit::dump_distributions()---  read_time_distribution[%d] = %ld", x, count); }
                }
            RCLCPP_INFO(log, "RoboClawUnit::dump_distributions()---  write_time_distribution  ----------");
            for (int x = 0; x < write_time_distribution.MAX_VALUE; x++)
                {
                    long count = write_time_distribution.getValue(x);
                    if (count > 0) { RCLCPP_INFO(log, "RoboClawUnit::dump_distributions()---  write_time_distribution[%d] = %ld", x, count); }
                }

            RCLCPP_INFO(log, "RoboClawUnit::dump_distributions()------------- done -------------------");
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~



    std::string RoboClawUnit::roboclaw_status_as_string(uint32_t status)
    {
        std::string status_str;
        if (status == 0) { return "Normal"; }
        if (status & 0x01) { status_str += "E-Stop, "; }
        if (status & 0x02) { status_str += "Temperature Error, "; }
        if (status & 0x04) { status_str += "Temperature 2 Error, "; }
        if (status & 0x08) { status_str += "Main Voltage High Error, "; }
        if (status & 0x10) { status_str += "Logic Voltage High Error, "; }
        if (status & 0x20) { status_str += "Logic Voltage Low Error, "; }
        if (status & 0x40) { status_str += "M1 Driver Fault Error, "; }
        if (status & 0x80) { status_str += "M2 Driver Fault Error, "; }
        if (status & 0x100) { status_str += "M1 Speed Error, ";}
        if (status & 0x200) { status_str += "M2 Speed Error, "; }
        if (status & 0x400) { status_str += "M1 Position Error, "; }
        if (status & 0x800) { status_str += "M2 Position Error, "; }
        if (status & 0x1000) { status_str += "M1 Current Error, "; }
        if (status & 0x2000) { status_str += "M2 Current Error, "; }
        if (status & 0x10000) { status_str += "M1 Over Current Warning, "; }
        if (status & 0x20000) { status_str += "M2 Over Current Warning, "; }
        if (status & 0x40000) { status_str += "Main Voltage High Warning, "; }
        if (status & 0x80000) { status_str += "Main Voltage Low Warning, "; }
        if (status & 0x100000) { status_str += "Temperature Warning, "; }
        if (status & 0x200000) { status_str += "Temperature 2 Warning, "; }
        if (status & 0x400000) { status_str += "S4 Signal Triggered, "; }
        if (status & 0x800000) { status_str += "S5 Signal Triggered, "; }
        if (status & 0x1000000) { status_str += "Speed Error Limit Warning, "; }
        if (status & 0x2000000) { status_str += "Position Error Limit Warning, "; }

        // Remove trailing comma and space
        if (!status_str.empty()) {
            status_str.erase(status_str.size() - 2);
        }
        return status_str;
    }

}       // namespace roboclaw_hardware_interface
