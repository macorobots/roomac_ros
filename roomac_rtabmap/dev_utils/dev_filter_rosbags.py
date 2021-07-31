#!/usr/bin/env python

import sys
import rosbag


def filterRosbag(inputRosbag, outputRosbag):
    topics_to_save = [
        "/camera/depth_registered/camera_info",
        "/camera/depth_registered/image_raw",
        "/camera/rgb/camera_info",
        "/camera/rgb/image_rect_color",
        "/odom",
    ]

    save_odom_tf = True

    with rosbag.Bag(outputRosbag, "w") as outbag:
        for topic, msg, t in rosbag.Bag(inputRosbag).read_messages():
            if topic == "/tf":
                if msg.transforms[0].header.frame_id != "map":
                    if save_odom_tf:
                        outbag.write(topic, msg, t)
                    else: 
                        pass


            elif topic in topics_to_save:
                outbag.write(topic, msg, t)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        filterRosbag(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        outputRosbag = str(sys.argv[1]) + "_filtered"
        print("Output rosbag not specified. Defaulting to: " + outputRosbag)
        filterRosbag(sys.argv[1], outputRosbag)
    else:
        print(
            "Invalid number of arguments. Usage: rosrun ... filter_rosbag.py inputRosbag [ouputRosbag]"
        )
