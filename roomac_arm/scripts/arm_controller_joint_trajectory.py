#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16, UInt8, Float32
import math
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryActionResult,
)


class Servo:
    def __init__(
        self, name, zero_position, lower_signal_bound, upper_signal_bound, scale_factor
    ):
        self.name = name
        self.position_pub = rospy.Publisher(
            name + "_position", UInt16, queue_size=5, latch=True
        )

        self.zero_position = zero_position
        self.last_position = None

        self.lower_signal_bound = lower_signal_bound
        self.upper_signal_bound = upper_signal_bound

        self.scale_factor = scale_factor

    def calculate_position_diff(self, new_position):
        return abs(new_position - self.last_position)

    def publish_position(self, position):
        position_msg = UInt16()
        position_msg.data = position
        self.position_pub.publish(position_msg)

    def transform_angle_to_signal(self, angle):
        return angle * self.scale_factor + self.zero_position

    def is_initialized(self):
        return not (self.last_position is None)

    def bound_angle(self, value):
        return max(self.lower_signal_bound, min(self.upper_signal_bound, value))

    def set_angle(self, angle):
        signal = self.bound_angle(self.transform_angle_to_signal(angle))
        self.publish_position(signal)
        self.last_position = angle


class AnalogServo(Servo):
    def __init__(
        self, name, zero_position, lower_signal_bound, upper_signal_bound, scale_factor
    ):
        Servo.__init__(
            self,
            name,
            zero_position,
            lower_signal_bound,
            upper_signal_bound,
            scale_factor,
        )
        self.speeds_pub = rospy.Publisher(
            self.name + "_speed", Float32, queue_size=5, latch=True
        )

    def publish_speed(self, speed):
        speed_msg = Float32()
        speed_msg.data = speed
        self.speeds_pub.publish(speed_msg)


class DigitalServo(Servo):
    def __init__(
        self, name, zero_position, lower_signal_bound, upper_signal_bound, scale_factor
    ):
        Servo.__init__(
            self,
            name,
            zero_position,
            lower_signal_bound,
            upper_signal_bound,
            scale_factor,
        )

        self.speeds_pub = rospy.Publisher(
            self.name + "_playtime", UInt8, queue_size=5, latch=True
        )
        self.publish_playtime(255)

    def bound_playtime(self, value):
        return max(0, min(255, value))

    def publish_playtime(self, playtime):
        playtime = self.bound_playtime(playtime)
        playtime_msg = UInt8()
        playtime_msg.data = playtime
        self.speeds_pub.publish(playtime_msg)


class ArmController:
    def __init__(self):
        self.zero_positions = rospy.get_param(
            "~zero_positions", [850, 320, 512, 1509, 1500, 650]
        )
        self.max_movement_time = rospy.get_param("~max_movement_time", 1000)
        self.analog_lower_signal_bound = rospy.get_param(
            "~analog_lower_signal_bound", 500
        )
        self.analog_upper_signal_bound = rospy.get_param(
            "~analog_upper_signal_bound", 2500
        )
        self.analog_lower_signal_bound_wrist = rospy.get_param(
            "~analog_lower_signal_bound_wrist", 600
        )
        self.analog_upper_signal_bound_wrist = rospy.get_param(
            "~analog_upper_signal_bound_wrist", 2400
        )
        self.digital_lower_signal_bound = rospy.get_param(
            "~digital_lower_signal_bound", 0
        )
        self.digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        self.wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        self.wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)
        self.analog_speed = rospy.get_param("~analog_speed", 10)
        self.wrist_speed = rospy.get_param("~wrist_speed", 4.0)
        self.analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self.change_threshold = rospy.get_param("~change_threshold", 0.01)
        self.max_speed = rospy.get_param("~max_speed", 0.005)

        self.joint_names = [
            "right_shoulder_pan",
            "right_shoulder_lift",
            "right_elbow",
            "right_wrist",
            "right_gripper_twist",
            "right_gripper",
        ]

        self.digital_joint_names = [
            self.joint_names[0],
            self.joint_names[1],
            self.joint_names[2],
        ]
        self.analog_joint_names = [
            self.joint_names[3],
            self.joint_names[4],
            self.joint_names[5],
        ]

        self.topic_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist",
            "wrist_twist",
            "gripper",
        ]

        self.analog_scale_factor_wrist = (
            self.wrist_signal_zero_position - self.wrist_signal_90_degrees
        ) / (90.0 * math.pi / 180.0)
        self.digital_scale_factor = (
            self.digital_upper_signal_bound - self.digital_lower_signal_bound
        ) / ((330.0 / 2.0) * math.pi / 180.0)
        self.analog_scale_factor = (
            self.analog_upper_signal_bound - self.analog_lower_signal_bound
        ) / (180.0 * math.pi / 180.0)

        self.scaling_factors = [
            self.digital_scale_factor,
            self.digital_scale_factor,
            self.digital_scale_factor / 2.0,  # no gear reduction
            self.analog_scale_factor_wrist,
            self.analog_scale_factor,
            self.analog_scale_factor,
        ]

        self.lower_signal_bounds = [
            self.digital_lower_signal_bound,
            self.digital_lower_signal_bound,
            self.digital_lower_signal_bound,
            self.analog_lower_signal_bound_wrist,
            self.analog_lower_signal_bound,
            self.analog_lower_signal_bound,
        ]

        self.upperSignalBounds = [
            self.digital_upper_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_upper_signal_bound,
            self.analog_upper_signal_bound_wrist,
            self.analog_upper_signal_bound,
            self.analog_upper_signal_bound,
        ]

        self.servos = {}

        for id in range(len(self.joint_names)):
            joint_name = self.joint_names[id]
            if joint_name in self.digital_joint_names:
                self.servos[joint_name] = DigitalServo(
                    self.topic_names[id],
                    self.zero_positions[id],
                    self.lower_signal_bounds[id],
                    self.upperSignalBounds[id],
                    self.scaling_factors[id],
                )
            elif joint_name in self.analog_joint_names:
                self.servos[joint_name] = AnalogServo(
                    self.topic_names[id],
                    self.zero_positions[id],
                    self.lower_signal_bounds[id],
                    self.upperSignalBounds[id],
                    self.scaling_factors[id],
                )

            self.servos[joint_name].set_angle(0.0)

        self.servos["right_wrist"].publish_speed(self.wrist_speed)

        self.action_server = actionlib.SimpleActionServer(
            "follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        self.feedback = FollowJointTrajectoryActionFeedback()
        self.result = FollowJointTrajectoryActionResult()

    def execute_cb(self, goal):
        rospy.loginfo("Goal received")
        i = 0

        # rospy.loginfo(goal)

        rospy.loginfo(
            "Number of points in trajectory " + str(len(goal.trajectory.points))
        )
        for trajectory_point in goal.trajectory.points:
            rospy.loginfo("Executing point " + str(i))
            i += 1

            movement_time = self.go_to_point(
                goal.trajectory.joint_names, trajectory_point.positions
            )
            rospy.sleep(rospy.Duration(movement_time / 100))

            # TODO feedback
            # self.feedback.feedback.actual = trajectory_point
            # self.feedback.feedback.joint_names = goal.trajectory.joint_names
            # self.action_server.publish_feedback(self.feedback)

            if self.action_server.is_preempt_requested():
                rospy.logwarn("follow_joint_trajectory Preempted")
                self.action_server.set_preempted()
                return

        rospy.loginfo("Finished goal execution")

        # TODO result
        # self.result.result.error_code = (
        #     FollowJointTrajectoryActionResult().result.SUCCESSFUL
        # )

        self.action_server.set_succeeded(self.result)

    def go_to_point(self, joint_names, positions):
        max_digital_angle_diff = 0
        max_analog_angle_diff = 0

        for id in range(len(joint_names)):
            joint_name = joint_names[id]

            if not joint_name in self.servos:
                continue

            if not self.servos[joint_name].is_initialized():
                rospy.logerr(joint_name + " not initialized")
                continue

            positionDiff = self.servos[joint_name].calculate_position_diff(
                positions[id]
            )

            if joint_name in self.digital_joint_names:
                if positionDiff > max_digital_angle_diff:
                    max_digital_angle_diff = positionDiff

            elif joint_name in self.analog_joint_names:
                if positionDiff > max_analog_angle_diff:
                    max_analog_angle_diff = positionDiff

        if (
            max_digital_angle_diff < self.change_threshold
            and max_analog_angle_diff < self.change_threshold
        ):
            return

        movement_time = self.calculate_movement_time(
            max_digital_angle_diff, max_analog_angle_diff
        )

        for joint_name in self.digital_joint_names:
            self.servos[joint_name].publish_playtime(movement_time)

        for id in range(len(joint_names)):

            joint_name = joint_names[id]

            if not joint_name in self.servos:
                continue

            self.servos[joint_name].set_angle(positions[id])

        return movement_time

    def calculate_movement_time(self, max_digital_angle_diff, max_analog_angle_diff):

        digital_movement_time = min(
            (max_digital_angle_diff / math.pi) * self.max_movement_time, 255
        )

        analog_movement_time = (
            (max_analog_angle_diff * self.analog_scale_factor) / self.analog_speed
        ) * self.analog_update_delay

        if digital_movement_time > analog_movement_time:
            movementTime = digital_movement_time
        else:
            movementTime = analog_movement_time

        if movementTime > self.max_movement_time:
            movementTime = self.max_movement_time

        return movementTime


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    controller = ArmController()
    rospy.spin()
