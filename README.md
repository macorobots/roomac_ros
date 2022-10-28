# Roomac ROS

[![Build and push docker image](https://github.com/macstepien/roomac_ros/actions/workflows/build_and_push_docker_image.yml/badge.svg)](https://github.com/macstepien/roomac_ros/actions/workflows/build_and_push_docker_image.yml)

Roomac is a low-cost autonomous mobile manipulation robot. It consists of a differential drive mobile base and a 5-DoF manipulator with a gripper. The costs of the whole construction summed up to around 550$ and using this platform I was able to prepare a proof-of-concept application - bringing a bottle to the user. In this repository you can find software and package configurations used to achieve this goal, as well as a simulation in Gazebo, which you can use to test it yourself.

<!-- ![roomac_logo](gihubreadme.jpg) -->
<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1ZuP4w7tW_VWIHfj-qy7BVNyVtaTG9-dw" height="400" />
  <img src="https://drive.google.com/uc?export=download&id=1rTuLH-XS4gVUvIicEU5ieaAnlDPMW0UM" height="400" />
</p>

Here you can see some gifs demonstrating the robot's capabilities (for more you can visit my [instagram page](https://www.instagram.com/macorobots/))

<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=1QqZTh-4e_rQb-nHz5QIrDAGeiYUWy7WU" height="300" />
</p>
<p align="center">
  <img src="https://drive.google.com/uc?export=download&id=12h62AJevcnTrKnjP31EyKXX0_Lz6-4fP" height="300" />
</p>

The recommended way of running simulation and real robot software is through docker containers, so first make sure that you have it installed.

## Simulation demo

If you would like to quickly run the application of bringing a bottle to the defined location you can use an already prepared map and config. 
First startup simulation container, there are two possibilities:

* Nvidia GPU config (make sure that you have Nvidia Container Runtime installed)
  ```
  docker compose -f docker/compose_simulation_demo_nvidia.yaml up
  ```

* other GPUs
  ```
  xhost local:root
  docker compose -f docker/compose_simulation_demo.yaml up
  ```

Nvidia configuration is recommended, as it has better performance.
<!-- > **Tip**
> If you have an Nvidia graphics card install Nvidia runtime (todo: describe) and use `compose_simulation_demo_nvidia.yaml` instead for better performance. -->


Then to execute fetching bottle run: 
```
docker exec roomac_simulation bash -c \
 "source /home/roomac/catkin_ws/devel/setup.bash &&
  rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal {}"
```

And that's it! The robot should navigate to the table, pick up the bottle and return to starting position. Please note that the accuracy of this operation isn't 100% and the robot may fail (especially detecting the robot's position in Kinect above table is prone to failure).

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
Alternatively run `xhost local:root` and use `compose_simulation_mapping.yaml` config if you don't have Nvidia GPU.

On the other terminal launch `teleop` to control the robot:
```
docker exec -it roomac_simulation bash -c \
 "source /home/roomac/catkin_ws/devel/setup.bash
  && roslaunch roomac_simulation teleop.launch"
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
 docker/compose_external_laptop_localization.yaml up
```

</td>
</tr>
    </tbody>
</table>

After launching everything drive the robot around. When the area is mapped simply kill launch files with Ctrl+C and the map will be saved to `roomac_data` directory.



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

Like in the previous step there is a config for other GPUs (`compose_simulation_localization.yaml`). Remember to first run `xhost local:root`

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
> All steps here will be referenced just if you had ROS installed natively. If you want to use only dockers instead you can use:
> ```
> docker exec roomac_simulation bash -c \
>  "source /home/roomac/catkin_ws/devel/setup.bash && HERE_COPY_COMMAND"
> ```
> Containers are run in network host mode, so you shouldn't have a problem running `rosservice` and `rostopic` locally (although remember that in some cases custom message types are used, which require built `roomac_msgs` package)

First it is necessary to save the home and table position, drive the robot to these locations and use services:
```
rosservice call /save_table_position
rosservice call /save_home_position
```

To start pick and bring action use:
```
rostopic pub /pick_and_bring/goal roomac_msgs/PickAndBringActionGoal {}
```

It is possible to cancel a goal using: 
```
rostopic pub /pick_and_bring/cancel actionlib_msgs/GoalID {}
```

It is also possible to use partial actions, only navigation using services: 
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

## Kinect in Docker
To use Kinect in docker, additionally there is a need to create udev rules on the host machine. Create `/etc/udev/rules.d/51-kinect.rules` with:
```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
```

## Troubleshooting

The date on Raspberry isn't synchronized:
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