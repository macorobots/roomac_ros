# Roomac ROS

![roomac_logo](.github/logo.png)

## Usage

### Running container on laptop

```
docker-compose -f compose_laptop.yaml run roomac
```

### Real robot mapping

On raspberry:
```
roslaunch roomac_base joy.launch
```

On laptop: 
```
rosrun rosserial_python serial_node.py
roslaunch roomac laptop_mapping_manual.launch
```
After recent problems with connecting base STM to Raspberry, it is necessary to connect it to laptop instead (that's why serial_node has to be run on laptop).

Visualization:
```
roslaunch roomac_rtabmap rviz_rtabmap_mapping.launch 
```

### Real robot localization

On raspberry:
```
roslaunch roomac_base joy.launch
```

On laptop: 
```
rosrun rosserial_python serial_node.py
roslaunch roomac laptop_localization_autonomy.launch
```

Visualization:
```
roslaunch roomac_move_base rviz_move_base.launch
```

Cancelling goal:
```
rostopic pub /move_base/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''" 
```

### Real robot picking
On raspberry:
```
rosrun rosserial_python serial_node.py
```

On laptop: 
```
roslaunch roomac external_laptop_nav_picking.launch
```
Then after everything launches to pick object:
```
rostopic pub /pick_object/goal roomac_msgs/PickObjectActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}" 
```
And to return to home position:
```
rosservice call /set_home_arm
```

### Real combined picking and autonomous navigation
On raspberry:
```
roslaunch roomac_base joy.launch
roslaunch roomac_arm serial.launch
```

On laptop: 
```
rosrun rosserial_python serial_node.py
roslaunch roomac laptop_nav_picking.launch
```

On external laptop: 
```
roslaunch roomac external_laptop_nav_picking.launch
```

Visualization:
```
roslaunch roomac_move_base rviz_move_base.launch
```

Services navigation:
```
rosservice call /save_table_position
rosservice call /save_home_position
rosservice call /go_to_table
rosservice call /go_to_home
```
Services picking:
```
rostopic pub /pick_object/goal roomac_msgs/PickObjectActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}" 
rosservice call /set_home_arm
```
Services combined (go to table, pick object, go to home position):
```
rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}" 
```

### Simulation mapping

```
roslaunch roomac_simulation simulation_mapping.launch
```
```
roslaunch roomac_simulation teleop.launch
```

### Simulation localization

```
roslaunch roomac_simulation simulation_localization_autonomy.launch
```

### Simulation picking
```
roslaunch roomac_simulation simulation_pick_place.launch
```
Then after everything launches to pick object:
```
rostopic pub /pick_object/goal roomac_msgs/PickObjectActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}" 
```
Cancelling:
```
rostopic pub /pick_object/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''" 
```

### Simulation combined picking and autonomous navigation
```
roslaunch roomac_simulation simulation_nav_pick.launch
```

### Arm precision testing

#### Real
On raspberry:
```
rosrun rosserial_python serial_node.py
```

On laptop: 
```
roslaunch roomac external_laptop_test_arm_precision.launch
```

Services:
```
rosservice call /go_to_test_point
rosservice call /set_home_arm
```

#### Simulation

```
roslaunch roomac_simulation simulation_test_arm_precision.launch
```

### Testing arm

On raspberry: 
```
roslaunch roomac_arm demo_arm_hardware.launch
```

On laptop:
```
roslaunch roomac_arm demo_arm_control_gui.launch
```

## Kinect in Docker
To use Kinect in docker, additionally there is a need to create udev rules on host machine. Create `/etc/udev/rules.d/51-kinect.rules` with:
```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
```

## Building dockers

### Docker for laptop

```
./docker/build_docker_laptop.sh
```

### Main Docker

```
./docker/build_docker.sh
```

## Troubleshooting

Date on Raspberry isn't synchronized:
```
sudo systemctl restart chrony.service
```
If it still doesn't work raspberry reboot may be necessary.

After creating roomac volume it is necessary to change ownership (rtabmap returns problems with permissions for saving map):
```
sudo chown -R roomac:roomac roomac_data/
```