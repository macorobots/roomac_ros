#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /home/roomac/catkin_ws/devel/setup.bash --extend
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/roomac/catkin_ws/src/roomac_ros/roomac_simulation/models/

exec "$@"