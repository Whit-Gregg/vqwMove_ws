#!/bin/bash
#
#  --cmake-clean-first
#
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF -DCMAKE_VERBOSE_MAKEFILE=1

