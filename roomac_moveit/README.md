# roomac_moveit

The package that contains launch files and configuration for running [MoveIt](https://moveit.ros.org/).

## Running

```
roslaunch roomac_moveit roomac_moveit.launch
```

### Nodes
 * `/move_group`

## ROS API short summary

### Action servers
 * `/move_group`

### Action clients
 * `/roomac/arm_position_controller/follow_joint_trajectory`
 * `/roomac/gripper_controller/follow_joint_trajectory`

### Subscriptions
 * `/tf`
 * `/tf_static`
 * `/camera_up/depth_registered/image_raw`

### Services
 * `/clear_octomap`
