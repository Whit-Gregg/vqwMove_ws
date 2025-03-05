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

#ifndef ROBOCLAW_HARDWARE_INTERFACE__MOTOR_JOINT_HPP_
#define ROBOCLAW_HARDWARE_INTERFACE__MOTOR_JOINT_HPP_

#include <math.h>

#include <array>
#include <hardware_interface/system_interface.hpp>
#include <memory>
#include <string>
#include "elapsedMillis.h"

namespace roboclaw_hardware_interface
{
    class MotorJoint
    {
      private:
        // The radians of rotation per tick
        const double ticks_per_radian_;

        const double ticks_per_meter_;

        // Desired velocity of the wheel in radians per second
        // double velocity_command_ = 0;
        hardware_interface::CommandInterface::SharedPtr velocity_command_interface;

        // Position of the wheel in radians
        // double position_state_ = 0;
        hardware_interface::StateInterface::SharedPtr position_state_interface;

        // Store the prior encoder count for updating position state
        int32_t prior_encoder_count_;
        bool    initialized_encoder_count_ = false;

        double position_state_ = 0.0;

        int32_t qppr;
        double  total_distance_meters = 0.0;
        int count_of_getTickRateCommand = 0;
        int32_t avg_encoder_count_change = 0;
        const int32_t avg_encoder_count_change_span = 5;

      public:
        typedef std::shared_ptr<MotorJoint> SharedPtr;

        // Name of the parent joint (from HardwareInfo)
        const std::string name;

        int32_t getProjectedEncoderCount() const
        {
            return prior_encoder_count_ + avg_encoder_count_change;
        }

        // Constructor
        MotorJoint(const std::string joint_name, const int32_t qppr, double wheel_diameter);

        void set_velocity_command_interface(hardware_interface::CommandInterface::SharedPtr velocity_command_interface_)
        {
            velocity_command_interface = velocity_command_interface_;
        }

        void set_position_state_interface(hardware_interface::StateInterface::SharedPtr position_state_interface_)
        {
            position_state_interface = position_state_interface_;
        }


        // the minimum velocity value is 0.00082, anything less than this is considered zero
        double get_velocity_command_value() const
        {
            double velocity = 0.0;
            bool   ok       = velocity_command_interface->get_value(velocity);
            if (!ok)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RoboclawHardwareInterface"),
                                 "MotorJoint[%s]::get_velocity_command_value() Failed to get velocity command value.", name.c_str());

                    throw std::runtime_error("Failed to get velocity command for " + name);
                }
            return velocity;
        }
        // void set_velocity_command(const double velocity_command)
        // {
        //   bool ok = velocity_command_interface->set_value(velocity_command);
        // }

        // double get_position_state() const
        // {
        //   double position_state=0.0;
        //   bool ok = position_state_interface->get_value(position_state);
        //   if (!ok) { throw std::runtime_error("Failed to get position state for " + name); }
        //   return position_state;
        // }

        // Return the tick rate required to execute the current velocity command
        int32_t getTickRateCommand();

        // Set the position given the current wheel encoder count
        void setPositionState(const int32_t &encoder_count);

        double get_total_distance_meters() const { return total_distance_meters; }

        // // // // Accessor methods to enable ros2_control to access joint interface pointers
        // // // inline double *getPositionStatePtr() { return &position_state_; }
        // // // inline double *getVelocityCommandPtr() { return &velocity_command_; }
    };
}       // namespace roboclaw_hardware_interface

#endif       // ROBOCLAW_HARDWARE_INTERFACE__MOTOR_JOINT_HPP_
