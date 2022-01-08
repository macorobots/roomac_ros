#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16, UInt8, Float32
import math


class Servo:
    def __init__(
        self, name, zeroPosition, lowerSignalBound, upperSignalBound, scaleFactor
    ):
        self.name = name
        self.positionPub = rospy.Publisher(
            name + "_position", UInt16, queue_size=5, latch=True
        )

        self.zeroPosition = zeroPosition
        self.lastPosition = None

        self.lowerSignalBound = lowerSignalBound
        self.upperSignalBound = upperSignalBound

        self.scaleFactor = scaleFactor

    def calculatePositionDiff(self, newPosition):
        return abs(newPosition - self.lastPosition)

    def publishPosition(self, position):
        positionMsg = UInt16()
        positionMsg.data = position
        self.positionPub.publish(positionMsg)

    def transformAngleToSignal(self, angle):
        return angle * self.scaleFactor + self.zeroPosition

    def isInitialized(self):
        return not (self.lastPosition is None)

    def boundAngle(self, value):
        return max(self.lowerSignalBound, min(self.upperSignalBound, value))

    def setAngle(self, angle):
        signal = self.boundAngle(self.transformAngleToSignal(angle))
        self.publishPosition(signal)
        self.lastPosition = angle


class AnalogServo(Servo):
    def __init__(
        self, name, zeroPosition, lowerSignalBound, upperSignalBound, scaleFactor
    ):
        Servo.__init__(
            self, name, zeroPosition, lowerSignalBound, upperSignalBound, scaleFactor
        )
        self.speedsPub = rospy.Publisher(
            self.name + "_speed", Float32, queue_size=5, latch=True
        )


class DigitalServo(Servo):
    def __init__(
        self, name, zeroPosition, lowerSignalBound, upperSignalBound, scaleFactor
    ):
        Servo.__init__(
            self, name, zeroPosition, lowerSignalBound, upperSignalBound, scaleFactor
        )

        self.speedsPub = rospy.Publisher(
            self.name + "_playtime", UInt8, queue_size=5, latch=True
        )
        self.publishPlaytime(255)

    def boundPlaytime(self, value):
        return max(0, min(255, value))

    def publishPlaytime(self, playtime):
        playtime = self.boundPlaytime(playtime)

        playtimeMsg = UInt8()
        playtimeMsg.data = playtime
        self.speedsPub.publish(playtimeMsg)


class ArmController:
    changeThreshold = 0.01
    maxSpeed = 0.005  # rad/s

    maxMovementTime = 1000
    # max value is 255, and it is rounded to it if excceding (in 10ms)

    analogUpdateDelay = 0.7
    # real value 0.7 (in units of 10ms as playtime in digitalServo)

    jointNames = [
        "right_shoulder_pan",
        "right_shoulder_lift",
        "right_elbow",
        "right_wrist",
        "right_gripper_twist",
        "right_gripper",
    ]

    digitalJointNames = [jointNames[0], jointNames[1], jointNames[2]]
    analogJointNames = [jointNames[3], jointNames[4], jointNames[5]]

    topicNames = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist",
        "wrist_twist",
        "gripper",
    ]

    zeroPositions = [850, 320, 512, 1770, 1500, 650]

    analogLowerSignalBound = 500
    analogUpperSignalBound = 2500
    digitalLowerSignalBound = 0
    digitalUpperSignalBound = 1024

    digitalScaleFactor = 1024 / ((330.0 / 2.0) * math.pi / 180.0)
    analogScaleFactor = 2000 / (180.0 * math.pi / 180.0)
    analogSpeed = 2  # Change in ms in signal per analogUpdateDelay
    scalingFactors = [
        digitalScaleFactor,
        digitalScaleFactor,
        digitalScaleFactor / 2.0,  # no gear reduction
        analogScaleFactor,
        analogScaleFactor,
        analogScaleFactor,
    ]

    servos = {}

    def __init__(self):
        self.jointsSub = rospy.Subscriber(
            "/joint_states", JointState, self.jointsStateCb, queue_size=1
        )

        for id in range(len(self.jointNames)):
            jointName = self.jointNames[id]
            if jointName in self.digitalJointNames:
                self.servos[jointName] = DigitalServo(
                    self.topicNames[id],
                    self.zeroPositions[id],
                    self.digitalLowerSignalBound,
                    self.digitalUpperSignalBound,
                    self.scalingFactors[id],
                )
            elif jointName in self.analogJointNames:
                self.servos[jointName] = AnalogServo(
                    self.topicNames[id],
                    self.zeroPositions[id],
                    self.analogLowerSignalBound,
                    self.analogUpperSignalBound,
                    self.scalingFactors[id],
                )

            self.servos[jointName].setAngle(0.0)

    def calculateMovementTime(self, maxDigitalAngleDiff, maxAnalogAngleDiff):

        digitalMovementTime = min(
            (maxDigitalAngleDiff / math.pi) * self.maxMovementTime, 255
        )

        analogMovementTime = (
            (maxAnalogAngleDiff * self.analogScaleFactor) / self.analogSpeed
        ) * self.analogUpdateDelay

        if digitalMovementTime > analogMovementTime:
            movementTime = digitalMovementTime
        else:
            movementTime = analogMovementTime

        if movementTime > self.maxMovementTime:
            movementTime = self.maxMovementTime

        return movementTime

    def jointsStateCb(self, state):

        maxDigitalAngleDiff = 0
        maxAnalogAngleDiff = 0

        for id in range(len(state.name)):
            jointName = state.name[id]

            if not jointName in self.servos:
                continue

            if not self.servos[jointName].isInitialized():
                rospy.logerr(jointName + " not initialized")
                continue

            positionDiff = self.servos[jointName].calculatePositionDiff(
                state.position[id]
            )

            if jointName in self.digitalJointNames:
                if positionDiff > maxDigitalAngleDiff:
                    maxDigitalAngleDiff = positionDiff

            elif jointName in self.analogJointNames:
                if positionDiff > maxAnalogAngleDiff:
                    maxAnalogAngleDiff = positionDiff

        if (
            maxDigitalAngleDiff < self.changeThreshold
            and maxAnalogAngleDiff < self.changeThreshold
        ):
            return

        movementTime = self.calculateMovementTime(
            maxDigitalAngleDiff, maxAnalogAngleDiff
        )

        for jointName in self.digitalJointNames:
            self.servos[jointName].publishPlaytime(movementTime)

        for id in range(len(state.name)):

            jointName = state.name[id]

            if not jointName in self.servos:
                continue

            self.servos[jointName].setAngle(state.position[id])

        rospy.sleep(rospy.Duration(movementTime / 100))


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    controller = ArmController()
    rospy.spin()
