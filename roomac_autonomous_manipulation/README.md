# roomac_autonomous_manipulation

This package implements object detection and picking up action necessary for autonomous manipulation. Object detection consists of analyzing point cloud data received from the Kinect sensor. The picking object manager provides an action that runs all operations necessary to pick up an object once the robot is near the table.

Test for this package consists of detecting object and table on the rosbag.

## Running

```
roslaunch roomac_autonomous_manipulation roomac_autonomous_manipulation.launch
```

### Nodes
 * `/object_detection_node` - parameters for this node are set in the `object_detection_config.yaml`
 * `/picking_object_manager` - it uses `move_group_commander`, so `roomac_moveit` has to be running. Parameters for this node are set in the `picking_object_manager_config.yaml` and either `picking_object_manager_config_real.yaml` or `picking_object_manager_config_simulation.yaml`.

## ROS API short summary

### Action servers
 * `/pick_object`

### Subscriptions
 * `/camera_up/depth_registered/points`

### Services
 * `/close_gripper`
 * `/open_gripper`
 * `/home_arm` - returns the arm to the home position
 * `/remove_object_from_scene` - clear objects from the MoveIt scene.

Communication with MoveIt is realized through [`move_group_commander`](http://docs.ros.org/en/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html).
