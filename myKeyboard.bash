#!/bin/bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=diffbot_base_controller/cmd_vel -p stamped:=true
#ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=diffbot_base_controller/cmd_vel
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
