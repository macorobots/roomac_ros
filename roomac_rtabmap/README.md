# roomac_rtabmap

The package that contains launch and configuration for [RTABMap SLAM package](http://wiki.ros.org/rtabmap_ros). 

## Running

To create a map use:
```
roslaunch roomac_rtabmap rtabmap.launch
```
To run localization mode (map already created):
```
roslaunch roomac_rtabmap rtabmap_localization.launch
```

### Nodes
 * `/rtabmap/rtabmap`
 * `/rtabmap/rgbd_sync`

## ROS API short summary

### Subscriptions
 * `/tf`
 * `/tf_static`
 * `/camera/color/camera_info`
 * `/camera/color/image_raw`
 * `/camera/depth/image_raw`

### Publications
 * `/tf` - `map` -> `odom` transform
 * `/rtabmap/octomap_grid`
