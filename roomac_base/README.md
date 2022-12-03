# roomac_base

Package providing low-level control of the robot's base - basically only launch configuration for serial communication and joystick control.

## Running

```
roslaunch roomac_base base_controller_hardware.launch 
```

### Nodes
 * `/serial_node_base` - `rosserial_python` node that connects to the upper microcontroller, [ros wiki page](http://wiki.ros.org/rosserial_python)
 * `/joy_node` - [ros wiki page](http://wiki.ros.org/joy)
 * `/teleop_node` - [ros wiki page](http://wiki.ros.org/teleop_twist_joy)

## ROS API short summary

### Subscriptions
 * `/cmd_vel`

### Publications
 * `/wheel_odom/position_raw`
 * `/imu/orientation_raw`
 * `/imu/angular_velocity_raw`
