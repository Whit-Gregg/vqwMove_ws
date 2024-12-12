#!/bin/bash

ros2 pkg create --license MIT --build-type ament_cmake --destination-directory src --maintainer-email whitgregg@hotmail.com --maintainer-name "Whit Gregg" --library-name $2 $1
