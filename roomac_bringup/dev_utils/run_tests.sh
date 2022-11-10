#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/roomac/catkin_ws/devel/setup.bash --extend
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/roomac/catkin_ws/src/roomac_ros/roomac_simulation/models/

dir_name=/home/roomac/catkin_ws/test_$(date '+%Y-%m-%d-%H-%M-%S')
mkdir $dir_name
for i in {1..10}
do
    echo "Test $i"
    rostest roomac_bringup pick_and_bring.test.launch > $dir_name/test_$i 2>&1
done