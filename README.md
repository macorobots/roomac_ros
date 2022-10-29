# Roomac ROS

<!-- TODO add distro information about badges -->
[![Build and push docker image](https://github.com/macstepien/roomac_ros/actions/workflows/build_and_push_docker_image.yml/badge.svg)](https://github.com/macstepien/roomac_ros/actions/workflows/build_and_push_docker_image.yml)
[![CI](https://github.com/macstepien/roomac_ros_workflow_test/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/macstepien/roomac_ros_workflow_test/actions/workflows/industrial_ci_action.yml)

Roomac is a low-cost autonomous mobile manipulation robot. It consists of differential drive mobile base and 5-DoF manipulator with gripper. Whole construction costed around 550$ and using this platform I was able to prepare proof-of-concept application - bringing bottle to the user. In this repository you can find software and package configurations used to achieve this goal, as well as simulation in Gazebo, which you can use to test it yourself.

<!-- ![roomac_logo](gihubreadme.jpg) -->
<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1ZuP4w7tW_VWIHfj-qy7BVNyVtaTG9-dw" height="400" />
  <img src="https://drive.google.com/uc?export=download&id=1rTuLH-XS4gVUvIicEU5ieaAnlDPMW0UM" height="400" />
</p>

<a href="https://www.instagram.com/macorobots/">
    <img height="50" src="https://www.vectorlogo.zone/logos/instagram/instagram-ar21.svg"/>
</a>


Recommended way of running simulation and real robot software is through docker containers, so first make sure that you have it installed.

## Simulation demo

If you would like to quickly run application of bringing bottle to defined location you can use already prepared map and config. 

### Nvidia GPU config
```
docker compose -f docker/compose_simulation_demo_nvidia.yaml up
```

### Other GPU config
First run:
```
xhost local:root
```
And then prepared compose: 
```
docker compose -f docker/compose_simulation_demo.yaml up
```

<!-- > **Tip**
> If you have NVidia graphics card install nvidia runtime (todo: describe) and use `compose_simulation_demo_nvidia.yaml` instead for better performance. -->


Then to execute fetching bottle run: 
```
docker exec roomac_simulation bash -c "source /home/roomac/catkin_ws/devel/setup.bash && rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal {}"
```

<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1QqZTh-4e_rQb-nHz5QIrDAGeiYUWy7WU" height="300" />
</p>
<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=12h62AJevcnTrKnjP31EyKXX0_Lz6-4fP" height="300" />
</p>

## Usage

In the following sections more details about running robot software will be described.

### Mapping

<!-- TODO: add gif with mapping -->

First for robot to navigate it is necessary to create map of the environment.

<!-- TODO describe creating volume -->

Setup:
<table>
    <thead>
        <tr>
            <th>Simulation</th>
            <th>Real robot</th>
        </tr>
    </thead>
    <tbody>
<tr>
<td rowspan=4>

First launch the docker container and run:
```bash
roslaunch roomac_simulation simulation_mapping.launch
```

On the other terminal launch `teleop` to control the robot:

```
roslaunch roomac_simulation teleop.launch
```

</td>

<td>
            
On raspberry:
```
roslaunch roomac raspberry.launch
```

</td>
</tr>

<tr>
</tr>
<tr>
<td>         

On laptop: 
```
docker-compose -f compose_laptop.yaml run roomac
```
```
roslaunch roomac laptop_mapping_manual.launch
```

</td>
</tr>
<tr>
<td>         

External laptop (used for visualization):
```
roslaunch roomac_rtabmap rviz_rtabmap_mapping.launch 
```

</td>
</tr>
    </tbody>
</table>

After launching everything drive robot around. When area is mapped simply kill launch files with Ctrl+C and map will be saved to `roomac_data` volume.



### Localization and manipulation

Setup:
<table>
    <thead>
        <tr>
            <th>Simulation</th>
            <th>Real robot</th>
        </tr>
    </thead>
    <tbody>
<tr>
<td rowspan=4>

First launch the docker container and run:
```
roslaunch roomac_simulation simulation_nav_pick.launch
```

</td>

<td>
            
On raspberry:
```
roslaunch roomac raspberry.launch
```

</td>
</tr>

<tr>
</tr>
<tr>
<td>         

On laptop:
```
docker-compose -f compose_laptop.yaml run roomac
```
```
roslaunch roomac laptop_nav_picking.launch
```

</td>
</tr>
<tr>
<td>         

External laptop:
```
roslaunch roomac external_laptop_nav_picking.launch
```

</td>
</tr>
    </tbody>
</table>


> **Tip**
> Simulation docker image comes with map and destinations of the environment, you can copy them and skip this step
First it is necessary to save home and table position, drive robot to these locations and use services:
```
rosservice call /save_table_position
rosservice call /save_home_position
```

To start pick and bring action use:
```
rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal {}
```

It is possible to cancel goal using: 
```
rostopic pub /pick_and_bring/cancel actionlib_msgs/GoalID {}
```

It is also possible to use partial actions, only navigation using services: 
```
rosservice call /go_to_table
rosservice call /go_to_home
```
or `2D Nav Goal` in RViz.
Cancelling goal:
```
rostopic pub /move_base/cancel actionlib_msgs/GoalID {}
```

Only picking object:
```
rostopic pub /pick_object/goal roomac_msgs/PickObjectActionGoal {}
```
Cancelling:
```
rostopic pub /pick_object/cancel actionlib_msgs/GoalID {}
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

<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1GaggM1wOW-irI4vqvEJq4knI8z77sjEJ" height="300" />
</p>