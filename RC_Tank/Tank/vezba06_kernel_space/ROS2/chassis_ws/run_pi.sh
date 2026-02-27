#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run wc_bridge twist_to_ackermann &
ros2 run wc_main simple_ackermann_controller
