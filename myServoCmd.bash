#!/bin/bash

ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [$1, $2, $3, $4]" --once

