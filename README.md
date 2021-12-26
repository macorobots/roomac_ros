# Roomac ROS packages

## Usage

### Real robot mapping

On raspberry:
```
roslaunch roomac raspberry_mapping_manual.launch
```

On laptop: 
```
roslaunch roomac laptop_mapping_manual.launch
```

### Real robot localization

On raspberry:
```
roslaunch roomac raspberry_localization_autonomy.launch
```

On laptop: 
```
roslaunch roomac laptop_localization_autonomy.launch
```

### Real robot picking
On raspberry:
```
rosrun rosserial_python serial_node.py
```

On laptop: 
```
roslaunch roomac_autonomous_manipulation real_picking.launch
```
Then after everything launches to pick object:
```
rosrun roomac_autonomous_manipulation real_pick_object.py
```


### Simulation picking
```
roslaunch roomac_simulation simulation_pick_place.launch
```
Then after everything launches to pick object:
```
rosrun roomac_autonomous_manipulation simulation_pick_object.py
```

## Kinect in Docker
To use Kinect in docker, additionally there is a need to create udev rules on host machine. Create `/etc/udev/rules.d/51-kinect.rules` with:
```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
```