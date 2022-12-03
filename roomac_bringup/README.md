# roomac_bringup

The package that provides main launch files for the fetch bottle application. `pick_and_bring_manager_node` implements this application as a sequence of driving up to the table position, calling pick up object action and returning to the home position. It additionally takes care of saving and loading home/table positions. 

Other nodes:
* `pose_publisher_node` - publishes robot position that can be visualized in ROSMobile
* `ros_mobile_bridge_node` - provides a bridge between ROSMobile topics and the robot interface, so that robot can be controlled using the mobile application
* `wheel_joint_publisher_node` - publishes current wheel positions as `JointState`

Tests for this package include a high-level integration test that checks the fetch bottle application in the Gazebo simulation. It is checked if the robot was able to return with the bottle to the defined location. 

## Running

For the robot's raspberry there is one launch:
```
roslaunch roomac_bringup raspberry.launch
```

The external laptop is used only during localization and picking: 
```
roslaunch roomac_bringup external_laptop_nav_picking.launch
```

For the robot's laptop there are two configurations, one for creating a map:
```
roslaunch roomac_bringup laptop_mapping_manual.launch
```
And second for running navigation and picking on created map
```
roslaunch roomac_bringup laptop_nav_picking.launch
```

### Nodes

 * `/pick_and_bring_manager`
 * `/wheel_joint_publisher`
 * `/pose_publisher`
 * `/ros_mobile_bridge`

## ROS API short summary

### Action servers
 * `/pick_and_bring`

### Action clients
 * `/move_base` - action provided by `move_base`, used for driving to defined goal
 * `/pick_object` - action provided by `picking_object_manager`

### Subscriptions
 * `/tf` - position of the robot on the map and checking if ARTag was detected and the robot can start picking up an object

### Services
 * `/save_home_position`
 * `/save_table_position`
 * `/go_to_table`
 * `/go_to_home`
