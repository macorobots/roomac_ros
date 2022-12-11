# roomac_arm

Package responsible for low-level control of the arm. It provides a controller that subscribes to  `FollowJointTrajectoryAction` and translates it into servo commands (`arm_joint_trajectory_controller_node`). Similarly, there is a separate controller for the gripper (`gripper_joint_trajectory_controller_node`). Feedback is published by `servo_joint_state_publisher_node` based on these commands - direct feedback from arm servos isn't provided.

`arm_joint_state_controller_node` is a controller that was used previously and it is based on `JointState` messages instead of `FollowJointTrajectoryAction`.

## Running

```
roslaunch roomac_arm roomac_arm.launch
```

### Nodes
 * `/arm_position_controller/serial_node_arm` - `rosserial_python` node that connects to the upper microcontroller
 * `/arm_position_controller/servo_joint_state_publisher`
 * `/arm_position_controller/arm_joint_trajectory_controller`
 * `/gripper_controller/gripper_joint_trajectory_controller`

For the parameters of `servo_joint_state_publisher`, `arm_joint_trajectory_controller` and `gripper_joint_trajectory_controller` refer to the configuration files in the `config` directory. `servo_controller_config.yaml` is used by both `gripper_joint_trajectory_controller` and `arm_joint_trajectory_controller`. `servo_joint_state_publisher` uses `arm_controller_config.yaml` and `gripper_controller_config.yaml`.

## ROS API short summary

### Action servers
 * `/arm_position_controller/follow_joint_trajectory`
 * `/gripper_controller/follow_joint_trajectory`

### Publications
 * `/arm_position_controller/joint_states_arm`
