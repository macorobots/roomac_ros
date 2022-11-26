# roomac_description

Package containing robot model.

## Running

There are separate models for running simulation and real model:
```
roslaunch roomac_description description.launch
```
```
roslaunch roomac_description description_simulation.launch
```

### Nodes

 * `/joint_state_publisher`
 * `/robot_state_publisher`

### Actions

### Subscriptions

 * `/arm_position_controller/joint_states_arm_stub`
 * `/wheel_joint_publisher/joint_states_wheels`

### Publications

 * `/joint_states`
 * `/tf`
 * `/tf_static`

### Services
