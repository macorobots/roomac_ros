# roomac_description

Package containing robot model.

## Running

There are separate launches for running simulation and real models:
```
roslaunch roomac_description description.launch
```
```
roslaunch roomac_description description_simulation.launch
```

### Nodes
 * `/joint_state_publisher`
 * `/robot_state_publisher`

## ROS API short summary

### Subscriptions
 * `/arm_position_controller/joint_states_arm`
 * `/wheel_joint_publisher/joint_states_wheels`

### Publications
 * `/joint_states`
 * `/tf`
 * `/tf_static`
