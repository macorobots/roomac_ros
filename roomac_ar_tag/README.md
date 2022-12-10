# roomac_ar_tag

Package responsible for detecting ARTags using `ar_track_alvar`. In the current setup it is used to detect the robot's position near the table (robot->camera transformation). 

After several tests and tweaking final configuration consists of a bundle of two markers with the additional assumption that the robot's marker is parallel to the camera. As `ar_track_alvar` estimates full 6DoF pose, `artag_parallel_tf_publisher_node` republishes transformations with fixed *roll* and *pitch* angle (to satisfy parallel assumption).

Package contains another configuration, which utilizes the `robot_localization` package to filter high-frequency noise in detections (currently it isn't used in the final setup).

Package test checks if detection works correctly on the recorded rosbag.

## Running

```
roslaunch roomac_ar_tag roomac_ar_tag.launch
```

### Nodes
Depending on the configuration, nodes used may differ, but the currently used configuration consists of:
 * `/ar_track_alvar` - you can find out more about this node on the [wiki page](http://wiki.ros.org/ar_track_alvar).
 * `/artag_parallel_tf_publisher_node` - reads the transform sent by `ar_track_alvar` and republishes it in respect to the robot's frame without *roll* and *pitch* angles. It can be configured, for example refer to `parallel_publisher_artag_config.yaml` in the `config` directory.

## ROS API short summary

### Subscriptions
 * `/camera_up/depth_registered/points`
 * `/camera_up/rgb/camera_info`

### Publications
 * `/tf` - transform from `artag_bundle_link` to `camera_up_link`
