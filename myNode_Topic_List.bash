#!/bin/bash

# Step 1: Execute a command and capture the output
output=$(ros2 node list)

# Step 2: Execute a second command for each line of the output
while IFS= read -r line; do
    ros2 node info "$line"
done <<< "$output"