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

#ifndef ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT_HPP_
#define ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT_HPP_

#include "roboclaw_hardware_interface/motor_joint.hpp"
#include <map>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/elapsedMillis.h"

namespace roboclaw_hardware_interface
{

    class RoboClawUnit
    {
      public:
        std::array<MotorJoint::SharedPtr, 2> joints;

        RoboClawUnit(roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1, MotorJoint::SharedPtr m2);

        // Read the encoder counts from the roboclaw and update position state
        void read();

        // Convert the velocity command to tick rate request and write to the roboclaw
        void write();

        // Read the firmware version from the roboclaw
        void read_firmware_version();

      private:
        roboclaw_serial::Interface::SharedPtr interface_;
        const uint8_t                         address_;
        long                                  write_count_ = 0;
        long                                  read_count_  = 0;

        std::int32_t m1_target_speed_in_ticks          = 0;
        std::int32_t m2_target_speed_in_ticks          = 0;
        std::int32_t m1_actual_speed_in_ticks          = 0;       // speed per interval, I just need to know if it is zero or not
        std::int32_t m2_actual_speed_in_ticks          = 0;       // speed per interval
        std::int32_t m1_actual_speed_in_ticks_previous = 0;       // speed per interval
        std::int32_t m2_actual_speed_in_ticks_previous = 0;       // speed per interval
        int          skip_count_                       = 0;
        const int    skip_count_max                    = 50;

        double m1_total_distance_meters = 0.0;
        double m2_total_distance_meters = 0.0;

        elapsedMillis elap_since_last_distance_report;
        uint32_t      elap_since_last_distance_report_span = 3000;       // 3 seconds
        void          report_distance_check();
        elapsedMillis elap_since_last_MainBatteryVoltage      = 0;
        uint32_t      elap_since_last_MainBatteryVoltage_span = 1000 * 55;       // 55 seconds

        bool should_skip();

        int main_battery_voltage_in_10ths_of_volts = 0;

        // RoboClaw serial driver messages
        roboclaw_serial::DriveM1M2WithSignedSpeed tick_rate_command_;
        roboclaw_serial::EncoderCounters          encoder_state_;
        roboclaw_serial::FirmwareVersion          firmware_version_;
        roboclaw_serial::MainBatteryVoltage       main_battery_voltage_;
    };
}       // namespace roboclaw_hardware_interface

#endif       // ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT_HPP_
