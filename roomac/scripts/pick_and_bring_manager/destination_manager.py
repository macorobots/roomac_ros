#!/usr/bin/env python

import yaml

import rospy

from geometry_msgs.msg import Pose2D


class DestinationManager:
    """Class that wraps using Pose2D named positions and saving/loading them
    in yaml file
    """

    def __init__(self, position_file):
        self._positions = {}

        self._position_file = position_file
        self.load_positions_from_file()

    def __getitem__(self, key):
        return self._positions[key]

    def __setitem__(self, key, value):
        self._positions[key] = value

    def load_positions_from_file(self):
        try:
            with open(self._position_file) as file:
                positions = yaml.load(file)
                for name, pos in positions.items():
                    self._positions[name] = Pose2D()
                    self._positions[name].x = pos[0]
                    self._positions[name].y = pos[1]
                    self._positions[name].theta = pos[2]

                    rospy.loginfo(
                        name + " position loaded: " + str(self._positions[name])
                    )
        except Exception as e:
            rospy.logerr("Error when trying to access positions file: " + str(e))

    def save_positions_to_file(self):
        positions_yaml_dict = {}
        for name in self._positions:
            if self._positions[name]:
                positions_yaml_dict[name] = [
                    self._positions[name].x,
                    self._positions[name].y,
                    self._positions[name].theta,
                ]
        with open(self._position_file, "w") as file:
            yaml.dump(positions_yaml_dict, file)
