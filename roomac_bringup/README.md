# roomac_bringup

Package that provides main launch files for fetch bottle application. `pick_and_bring_manager_node` implements this application as sequence of driving up to table position, calling pick up object action and returning to home position. It additionally takes care of saving and loading home/table positions. 

Other nodes:
* `pose_publisher_node` - publishes robot position that can be visualized in ROSMobile
* `ros_mobile_bridge_node` - provides bridge between ROSMobile topics and robot interface, so that robot can be controlled using mobile application
* `wheel_joint_publisher_node` - node that publishes current wheel positions as `JointState`

## Running

For robot's raspberry there is one launch:
```
roslaunch roomac_bringup raspberry.launch
```

External laptop is used only during localization and picking: 
```
roslaunch roomac_bringup external_laptop_nav_picking.launch
```

For robot's laptop there are two configurations, one for creating map:
```
roslaunch roomac_bringup laptop_mapping_manual.launch
```
And second for running navigation and picking on created map
```
roslaunch roomac_bringup laptop_nav_picking.launch
```

### Nodes

 * `/wheel_joint_publisher`
 * `/pick_and_bring_manager`
 * `/pose_publisher`
 * `/ros_mobile_bridge`

### Actions

### Subscriptions

### Publications

### Services
