# roomac_kinect

Package containing launches for upper and base Kinect sensors.

## Running

There are two separate launches, one for the upper Kinect (mounted above the table) and one for the base Kinect (mounted on the robot): 
```
roslaunch roomac_kinect base_kinect.launch
```
```
roslaunch roomac_kinect upper_kinect.launch
```

It launches the libfreenect driver using the [freenect_launch](http://wiki.ros.org/freenect_launch) package.

## ROS API short summary

### Publications
 * `/camera_up/depth_registered/points`
 * `/camera_up/rgb/camera_info`
 * `/camera/depth_registered/points`
 * `/camera/rgb/camera_info`
