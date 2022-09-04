# Roomac ROS

[![Build and push docker image](https://github.com/macstepien/roomac_ros/actions/workflows/build_and_push_docker_image.yml/badge.svg)](https://github.com/macstepien/roomac_ros/actions/workflows/build_and_push_docker_image.yml)

![roomac_logo](.github/logo.png)

## Usage

### Mapping

#### Real robot setup

On raspberry:
```
roslaunch roomac raspberry.launch
```

On laptop: 
```
docker-compose -f compose_laptop.yaml run roomac
```
```
roslaunch roomac laptop_mapping_manual.launch
```

External laptop (used for visualization):
```
roslaunch roomac_rtabmap rviz_rtabmap_mapping.launch 
```

#### Simulation setup

```
roslaunch roomac_simulation simulation_mapping.launch
```
```
roslaunch roomac_simulation teleop.launch
```

#### Usage

Drive robot around, after area is mapped simply kill launch files with Ctrl+C and map will be saved.

### Localization and manipulation

#### Real robot setup


On raspberry:
```
roslaunch roomac raspberry.launch
```

On laptop:
```
docker-compose -f compose_laptop.yaml run roomac
```
```
roslaunch roomac laptop_nav_picking.launch
```

External laptop:
```
roslaunch roomac external_laptop_nav_picking.launch
```


#### Simulation setup

```
roslaunch roomac_simulation simulation_nav_pick.launch
```

#### Usage

First it is necessary to save home and table position, drive robot to these locations and use services:
```
rosservice call /save_table_position
rosservice call /save_home_position
```

Then to start pick and bring action use:
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
It is possible to cancel goal using: 
```
rostopic pub /pick_and_bring/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''" 
```

It is also possible to use partial actions, only navigation using services: 
```
rosservice call /go_to_table
rosservice call /go_to_home
```
or `2D Nav Goal` in RViz.
Cancelling goal:
```
rostopic pub /move_base/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''" 
```

Only picking object:
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
And to return to home position:
```
rosservice call /home_arm
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

```
docker compose -f docker/compose.yaml build
```
To build image for laptop use `compose_laptop.yaml` instead.

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