# roomac_move_base

The package that contains the launch file and parameters for running autonomous navigation stack using `move_base`.

## Running

```
roslaunch roomac_move_base move_base.launch
```

### Nodes
 * `move_base_node` - [ros wiki page](http://wiki.ros.org/move_base)

## ROS API short summary

### Action servers
 * `/move_base`

### Subscriptions
 * `/tf` - position of the robot in the `odom` and `map` frames
 * `/tf_static`
 * `/camera/depth_registered/points` - data from Kinect which is used by the STVL and marked on the local costmap
 * `/odometry/filtered`
 * `/rtabmap/octomap_grid` - grid map from RTABMap

### Publications
 * `/cmd_vel`
