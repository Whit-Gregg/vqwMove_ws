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

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  MotorJoint::MotorJoint(const std::string joint_name, const int32_t qppr_, double wheel_diameter)
      : ticks_per_radian_(static_cast<double>(qppr_) / (M_PI * 2.0)),
        ticks_per_meter_(static_cast<double>(qppr_) / (wheel_diameter * M_PI)),
        qppr(qppr_),
        name(joint_name)
  {
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  int32_t MotorJoint::getTickRateCommand() const
  {
    return static_cast<int32_t>(velocity_command_ * ticks_per_radian_);
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  // Set the position given the current wheel encoder count
  void MotorJoint::setPositionState(const int32_t &encoder_count)
  {
    if (initialized_encoder_count_)
    {
      // Update the joint angle
      position_state_ +=
          static_cast<double>(encoder_count - prior_encoder_count_) / ticks_per_radian_;
      // Update the total distance traveled
      total_distance_meters += std::abs(static_cast<double>(encoder_count - prior_encoder_count_) / ticks_per_meter_);
    }
    else
    {
      initialized_encoder_count_ = true;
    }
    // Store the prior encoder count for next time
    prior_encoder_count_ = encoder_count;
  }
} // namespace roboclaw_hardware_interface
