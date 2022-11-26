# roomac_ar_tag

Package responsible for detecting ar tags using `ar_track_alvar`. In current setup it is used to detect robot's position near table (robot->camera transformation). 

After several tests and tweaking final configuration consists of bundle of two markers with additional assumption  that robot's marker is parallel to the camera. As `ar_track_alvar` estimates full 6DoF pose `artag_parallel_tf_publisher_node` republishes transformations with fixed `roll` and `pitch` angle (to satisfy parallel assumption).

Package contains another configuration, which utilizes `robot_localization` package to filter high frequency noise in detections (currently it isn't used in the final setup).

Package test checks if detection works correctly on recorded rosbag.

## Running

```
roslaunch roomac_ar_tag roomac_ar_tag.launch
```

Depending on the configuration, nodes used may differ, but currently used configuration consists of:

### Nodes
 * `/ar_track_alvar`
 * `/artag_parallel_tf_publisher_node`

### Subscriptions
 * `/camera_up/depth_registered/points`
 * `/camera_up/rgb/camera_info`

### Publications
 * `/tf` - transform from `artag_bundle_link` to `camera_up_link`

> Please note that this are only the most crucial topics, there also several others, that could be useful for debugging, you can find them using `rostopic list` and `rosnode info`
