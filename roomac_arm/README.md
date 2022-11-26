# roomac_arm

Package responsible for low level control of the arm. It provides controller that subscribes to  `FollowJointTrajectoryAction` and translates it into servo commands (`arm_joint_trajectory_controller_node`). Similarly there is a separate controller for gripper (`gripper_joint_trajectory_controller_node`). Feedback is published by `servo_joint_state_publisher_node` based on this commands - direct feedback from arm servos isn't provided.

`arm_joint_state_controller_node` is a controller that was used previously and it is based on `JointState` messages instead of `FollowJointTrajectoryAction`.

## Running

```
roslaunch roomac_arm roomac_arm.launch
```

### Nodes
 * `/arm_position_controller/serial_node_arm`
 * `/arm_position_controller/servo_joint_state_publisher`
 * `/arm_position_controller/arm_joint_trajectory_controller`
 * `/gripper_controller/gripper_joint_trajectory_controller`


### Actions
 * `/arm_position_controller/follow_joint_trajectory`
 * `/gripper_controller/follow_joint_trajectory`

### Subscriptions

### Publications

 * `/arm_position_controller/joint_states_arm_stub`

Commands sent to servos (they are subscribed by node running on upper microcontroller)
 * `/arm_position_controller/elbow_cmd`
 * `/arm_position_controller/gripper_cmd`
 * `/arm_position_controller/shoulder_lift_cmd`
 * `/arm_position_controller/shoulder_pan_cmd`
 * `/arm_position_controller/wrist_cmd`
 * `/arm_position_controller/wrist_twist_cmd`
