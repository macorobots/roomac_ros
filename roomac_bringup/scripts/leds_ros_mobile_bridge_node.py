#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2023, Maciej Stepien, All rights reserved.
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
#  Authors: Maciej Stepien

import rospy

from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import Point, Quaternion

from enum import Enum


class EyesMode(Enum):
    STATIC_COLOR = 0
    RAINBOW = 1
    BLINKING = 2


class BodyMode(Enum):
    STATIC_COLOR = 0
    RAINBOW = 1


class ValueModifier:
    def __init__(self, object, field_name, value):
        self._object = object
        self._field_name = field_name
        self._value = value

    def modify_cb(self, msg):
        if msg.data:
            self._object[self._field_name] += self._value


class LedController:
    def __init__(
        self, color, brightness, color_type, color_to_msg, topic_prefix, mode, mode_type
    ):
        self._color = color
        self._brightness = brightness
        self._color_to_msg = color_to_msg
        self._mode = mode
        self._mode_type = mode_type

        self._color_pub = rospy.Publisher(
            "/" + topic_prefix + "/color", color_type, queue_size=10, latch=True
        )
        self._brightness_pub = rospy.Publisher(
            "/" + topic_prefix + "/brightness", UInt8, queue_size=10, latch=True
        )
        self._mode_pub = rospy.Publisher(
            "/" + topic_prefix + "/mode", UInt8, queue_size=10, latch=True
        )

        value_operations = [
            ("_increase_1", 1),
            ("_increase_10", 10),
            ("_decrease_1", -1),
            ("_decrease_10", -10),
        ]

        self._values_modifiers = []
        self._ros_mobile_subs = []

        fields = [c for c in self._color] + ["brightness"]

        for c in fields:
            for topic, op in value_operations:
                self._values_modifiers.append(ValueModifier(self, c, op))
                self._ros_mobile_subs.append(
                    rospy.Subscriber(
                        "/" + topic_prefix + "_" + c + topic,
                        Bool,
                        self._values_modifiers[-1].modify_cb,
                    )
                )

        self._values_modifiers.append(ValueModifier(self, c, op))
        self._ros_mobile_subs.append(
            rospy.Subscriber(
                "/" + topic_prefix + "_" + c + topic,
                Bool,
                self._values_modifiers[-1].modify_cb,
            )
        )

        self.publish_color()
        self.publish_brightness()
        self.publish_mode()

    def __getitem__(self, key):
        if key == "brightness":
            return self._brightness
        elif key == "mode":
            return self._mode
        else:
            return self._color[key]

    def __setitem__(self, key, value):
        if key == "brightness":
            self._brightness = max(min(value, 255), 0)
            self.publish_brightness()
        elif key == "mode":
            self._mode = value % len(self._mode_type)
            self.publish_mode()
        elif key in self._color:
            self._color[key] = max(min(value, 255), 0)
            self.publish_color()

    def publish_color(self):
        self._color_pub.publish(self._color_to_msg(self._color))

    def publish_brightness(self):
        self._brightness_pub.publish(UInt8(data=self._brightness))
    
    def publish_mode(self):
        self._mode_pub.publish(UInt8(data=self._mode))


if __name__ == "__main__":
    rospy.init_node("ros_mobile_leds_bridge")

    eyes_led_controller = LedController(
        color={"r": 255, "g": 255, "b": 255},
        brightness=40,
        color_type=Point,
        color_to_msg=lambda color: Point(x=color["r"], y=color["g"], z=color["b"]),
        topic_prefix="eyes",
        mode=EyesMode.STATIC_COLOR.value,
        mode_type=EyesMode,
    )

    body_led_controller = LedController(
        color={"r": 255, "g": 255, "b": 255, "w": 255},
        brightness=40,
        color_type=Quaternion,
        color_to_msg=lambda color: Quaternion(
            x=color["r"], y=color["g"], z=color["b"], w=color["w"]
        ),
        topic_prefix="body",
        mode=BodyMode.STATIC_COLOR.value,
        mode_type=BodyMode,
    )

    rospy.spin()
