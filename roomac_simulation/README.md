# roomac_simulation

Gazebo simulation configuration for the roomac robot.

## Running

To create a map of the environment run:
```
roslaunch roomac_simulation simulation_mapping.launch
```

To run navigation and picking on previously created map use:
```
roslaunch roomac_simulation simulation_localization_fetching.launch
```

There is another configuration, which spawns the robot next to the table and only packages necessary for manipulation are run in this case.
```
roslaunch roomac_simulation simulation_only_picking.launch
```
