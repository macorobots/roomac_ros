# roomac_autonomous_manipulation

This package implements object detection and picking up action necessary for autonomous manipulation. Object detection consists of analyzing point cloud data received from Kinect sensor. Picking object manager provides action that runs all operations necessary to pick up object once robot drives up to the table.

Test for this package consists of detecting object and table on rosbag.


## Running

```
roslaunch roomac_autonomous_manipulation roomac_autonomous_manipulation.launch
```

### Nodes
 * `/object_detection_node`
 * `/picking_object_manager` (uses move_group_commander, so roomac_moveit has to be running)

<!-- ### Publications
 /attached_collision_object [moveit_msgs/AttachedCollisionObject]
 /collision_object [moveit_msgs/CollisionObject]
 /move_group/cancel [actionlib_msgs/GoalID] -->
### Actions
 * `/pick_object`

### Services

 * `/detect_table_and_object`
  
 * `/close_gripper`
 * `/home_arm`
 * `/open_gripper`
 * `/remove_object_from_scene`
