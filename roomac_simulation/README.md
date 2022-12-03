# roomac_simulation

Gazebo simulation configuration for the roomac robot.

## Running

To create a map of the environment run:
```
roslaunch roomac_simulation simulation_mapping.launch
```

To run navigation and picking on previously created map use:
```
roslaunch roomac_simulation simulation_nav_pick.launch
```

There is another configuration, which spawns the robot next to the table, only MoveIt is run then, it can be used to check only how picking objects works
```
roslaunch roomac_simulation simulation_only_picking.launch
```
