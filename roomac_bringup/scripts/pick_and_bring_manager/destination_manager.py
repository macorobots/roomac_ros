#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stępień, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  Authors: Maciej Stępień

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
