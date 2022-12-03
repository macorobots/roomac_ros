# roomac_sensor_fusion

Package responsible for publishing odometry information. `republish_odom_imu_msgs_node` node republishes lightweight messages containing information from IMU and wheel odometry to standard ones that can be used by `robot_localization`. Also config and launch for `robot_localization` are provided in this package.

## Running

```
roslaunch roomac_sensor_fusion robot_localization.launch
```

### Nodes
 * `/republish_odom_imu_msgs_node`
 * `/ekf_localization_node`

## ROS API short summary

### Subscriptions
 * `/wheel_odom/position_raw`
 * `/imu/orientation_raw`
 * `/tf_static` - position of the IMU sensor

### Publications
 * `/odometry/filtered`
 * `/tf` - `base_link`->`odom`
