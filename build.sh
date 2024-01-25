#!/bin/bash

source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --continue-on-error --symlink-install
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --continue-on-error --symlink-install
