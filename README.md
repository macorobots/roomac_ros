# Roomac ROS

[![Build and push docker image](https://github.com/macorobots/roomac_ros/actions/workflows/build_and_push_docker_image.yml/badge.svg)](https://github.com/macorobots/roomac_ros/actions/workflows/build_and_push_docker_image.yml)
[![CI](https://github.com/macorobots/roomac_ros/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/macorobots/roomac_ros/actions/workflows/industrial_ci_action.yml)
![ROS Distro](https://img.shields.io/badge/ROS%20Distro-melodic-informational)
[![Docker](https://img.shields.io/badge/dockerhub-images-informational)](https://hub.docker.com/repository/docker/macorobots/roomac)

Roomac is a low-cost autonomous mobile manipulation robot that consists of a differential drive mobile base and a 5-DoF manipulator with a gripper. The costs of the whole construction summed up to around 550$ and using this platform I was able to prepare a proof-of-concept application - fetching a bottle to the user. In this repository you can find software and package configurations used to achieve this goal, as well as a simulation in Gazebo, which you can use to test it yourself.

<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1ZuP4w7tW_VWIHfj-qy7BVNyVtaTG9-dw" height="400" />
  <img src="https://drive.google.com/uc?export=download&id=1rTuLH-XS4gVUvIicEU5ieaAnlDPMW0UM" height="400" />
</p>

Here you can see some gifs demonstrating the robot's capabilities (and [here](https://youtu.be/toHzFQhAP44) you can watch the full demo).

<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1QqZTh-4e_rQb-nHz5QIrDAGeiYUWy7WU" height="300" />
</p>
<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=12h62AJevcnTrKnjP31EyKXX0_Lz6-4fP" height="300" />
</p>

## Simulation demo

> The recommended way of running simulation and real robot software is through docker containers, so first make sure that you have it installed.

If you would like to quickly run the application of fetching a bottle to the defined location you can use an already prepared map and config.
First start simulation container, there are two possibilities:

* Nvidia GPU config (make sure that you have Nvidia Container Runtime installed)
  ```
  docker compose -f docker/compose_simulation_demo_nvidia.yaml up
  ```

* other GPUs
  ```
  docker compose -f docker/compose_simulation_demo.yaml up
  ```

Nvidia configuration is recommended, as it has better performance.

After everything launches, to execute the fetch bottle action run: 
```
docker exec roomac_simulation bash -c \
 "source /home/roomac/catkin_ws/devel/setup.bash &&
  rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal {}"
```

And that's it! The robot should navigate to the table, pick up the bottle and return to the starting position. Please note that the accuracy of this operation isn't 100% and the robot may fail (especially detecting the robot's position with Kinect above table is prone to failure).

## Packages

Packages in the repository can be divided into 3 sections:

1. Manipulation:
   * `roomac_arm` - low-level hardware controller that provides `FollowJointTrajectoryAction` implementation.
   * `roomac_moveit` - configuration for MoveIt package, destination points for end effector are requested and it calculates collision-free trajectory that is later executed by arm controller from `roomac_arm` package.
   * `roomac_ar_tag` - provides a position of the robot in the upper camera coordinate frame (necessary to determine what is the position of the object in respect to the robot).
   * `roomac_autonomous_manipulation` - provides object detection and pick-up object action (sequence of destination points necessary to pick up an object).

2. Navigation:
   * `roomac_base` - provides communication with the base microcontroller and joystick control.
   * `roomac_sensor_fusion` - provides fused odometry information based on wheel odometry and IMU data.
   * `roomac_rtabmap` - RTABMap configuration (SLAM and localization).
   * `roomac_move_base` - configuration for move_base package - autonomous navigation.

3. General packages:
   * `roomac` - metapackage providing source dependencies.
   * `roomac_bringup` - launch configurations and action implementation for running the whole fetch bottle application.
   * `roomac_description` - URDF model of the robot.
   * `roomac_kinect` - launch files for Kinect sensors (used both in navigation and manipulation)
   * `roomac_msgs` - custom message, action and service definitions.
   * `roomac_simulation` - configurations and launch files used in Gazebo simulation.
   * `roomac_utils` - custom code libraries used in other packages.

For more details refer to the README file in each package.

## Further reading

To find out more about this project you can check out my [**master's thesis**](https://raw.githubusercontent.com/macstepien/macstepien.github.io/master/files/masters_thesis_maciej_stepien.pdf).

## Usage

In the following sections more details about running robot software will be described.

### Mapping

> **Tip**
> Simulation docker image comes with a map and destinations of the environment, you can copy them (or pass as parameters just like in the demo) and skip this step

First for the robot to navigate it is necessary to create a map of the environment.

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
docker compose -f \
 docker/compose_simulation_mapping_nvidia.yaml up
```
Alternatively use the `compose_simulation_mapping.yaml` config if you don't have Nvidia GPU.

On the other terminal launch `teleop` to control the robot:
```
docker exec -it roomac_simulation bash -c \
 "source /home/roomac/catkin_ws/devel/setup.bash &&
  roslaunch roomac_simulation teleop.launch"
```

</td>

<td>
            
On raspberry:
```
roslaunch roomac_bringup raspberry.launch
```

</td>
</tr>

<tr>
</tr>
<tr>
<td>         

On laptop: 
```
docker compose -f \
 docker/compose_laptop_mapping.yaml up
```

</td>
</tr>
<tr>
<td>         


External laptop (used for visualization):
```
docker compose -f \
 docker/compose_external_laptop_mapping.yaml up
```

</td>
</tr>
    </tbody>
</table>

After launching everything drive the robot around. When the area is mapped simply kill docker with Ctrl+C and the map will be saved in the `roomac_data` directory.

> **Tip** Currently there is a problem with creating larger maps in simulation, so driving for too long may break the map.

> **Tip** Make sure to also map starting spot, as the robot is allowed to move autonomously only on the mapped area.

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
docker compose -f \
 docker/compose_simulation_localization_nvidia.yaml up
```

Like in the previous step there is a config for other GPUs (`compose_simulation_localization.yaml`).

</td>

<td>
            
On raspberry:
```
roslaunch roomac_bringup raspberry.launch
```

</td>
</tr>

<tr>
</tr>
<tr>
<td>         

On laptop:
```
docker compose -f \
 docker/compose_laptop_localization.yaml up
```

</td>
</tr>
<tr>
<td>         

External laptop:
```
docker compose -f \
 docker/compose_external_laptop_localization.yaml up
```

</td>
</tr>
    </tbody>
</table>

> **Tip**
> All following steps will assume that you have ROS installed natively. If you want to use only dockers, instead you can use:
> ```
> docker exec roomac_simulation bash -c \
>  "source /home/roomac/catkin_ws/devel/setup.bash && HERE_COPY_COMMAND"
> ```
> Containers are run in network host mode, so you shouldn't have a problem running `rosservice` and `rostopic` locally (although remember that in some cases custom message types are used, which require built `roomac_msgs` package)

First it is necessary to save the home and table position, drive the robot to these locations (for example using `2D Nav Goal` or `teleop`) and use services:
```
rosservice call /save_table_position
rosservice call /save_home_position
```

> **Tip** It may be necessary to fine-tune table position, to get proper detection on the upper Kinect sensor.

To start pick and bring action use:
```
rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal {}
```

It is possible to cancel a goal using: 
```
rostopic pub /pick_and_bring/cancel actionlib_msgs/GoalID {}
```

It is also possible to use partial actions, navigation using services: 
```
rosservice call /go_to_table
rosservice call /go_to_home
```
or `2D Nav Goal` in RViz.

Canceling goal:
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
And to return the arm to the home position:
```
rosservice call /home_arm
```

---

<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1GaggM1wOW-irI4vqvEJq4knI8z77sjEJ" height="300" />
</p>
