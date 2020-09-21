#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16, UInt8, Float32
import math

class ArmController:
    lastState = None
    changeThreshold = 0.01
    maxSpeed = 0.005 #rad/s
    analogUpdateDelay = 1. #in units of 10ms as playtime in digitalServo
    jointNames = ["shoulder_pan", "shoulder_lift", "elbow", "wrist", "wrist_twist", "gripper"]
    servoIds = {jointNames[0]:0, jointNames[1]:1, jointNames[2]:2, jointNames[3]:3, jointNames[4]:4, jointNames[5]:5}
    positionPub = []
    speedsPub = []

    
    zeroPositions = [850, 320, 512, 1770, 1500, 650]
    
    analogLowerSignalBound = 500
    analogUpperSignalBound = 2500
    digitalLowerSignalBound = 0
    digitalUpperSignalBound = 1024

    lowerSignalBound = [digitalLowerSignalBound, digitalLowerSignalBound, digitalLowerSignalBound, analogLowerSignalBound, analogLowerSignalBound, analogLowerSignalBound]
    upperSignalBound = [digitalUpperSignalBound, digitalUpperSignalBound, digitalUpperSignalBound, analogUpperSignalBound, analogUpperSignalBound, analogUpperSignalBound]

    digitalScaleFactor = 1024/((330./2.)*math.pi/180.)
    analogScaleFactor = 2000/(180.*math.pi/180.)
    scalingFactors = [digitalScaleFactor, digitalScaleFactor, digitalScaleFactor/2., analogScaleFactor, analogScaleFactor, analogScaleFactor]
    
    def __init__(self):
        self.jointsSub = rospy.Subscriber("/joint_states", JointState, self.jointsStateCb)

        for i in range(len(self.jointNames)):
            self.positionPub.append(rospy.Publisher(self.jointNames[i]+"_position", UInt16, queue_size=5))
        
        for i in range(3):
            self.speedsPub.append(rospy.Publisher(self.jointNames[i]+"_playtime", UInt8, queue_size=5))
        
        for i in range(3, len(self.jointNames)):
            self.speedsPub.append(rospy.Publisher(self.jointNames[i]+"_speed", Float32, queue_size=5))
        


    def transformAngleToSignal(self, angle, id):
        return angle*self.scalingFactors[id] + self.zeroPositions[id]

    def bound(self, low, high, value):
        return max(low, min(high, value))

    def jointsStateCb(self, state):
        # state = JointState()
        if not self.lastState:
            self.lastState = state
            return

        maxAngleDiff = 0
        for i in range(len(state.position)):
            diff = abs(state.position[i] - self.lastState.position[i])
            if diff > maxAngleDiff:
                maxAngleDiff = diff
        
        if maxAngleDiff < self.changeThreshold:
            return
        
        movementTime = maxAngleDiff/self.maxSpeed

        digitalPlaytime = movementTime

        analogUpdateTimes = (movementTime/self.analogUpdateDelay)
        diffWrist = abs(state.position[self.servoIds["wrist"]] - self.lastState.position[self.servoIds["wrist"]])*self.scalingFactors[self.servoIds["wrist"]]
        analogSpeedWrist = diffWrist/analogUpdateTimes

        diffWristTwist = abs(state.position[self.servoIds["wrist_twist"]] - self.lastState.position[self.servoIds["wrist_twist"]])*self.scalingFactors[self.servoIds["wrist_twist"]]
        analogSpeedWristTwist = diffWristTwist/analogUpdateTimes

        diffGripper = abs(state.position[self.servoIds["gripper"]] - self.lastState.position[self.servoIds["gripper"]])*self.scalingFactors[self.servoIds["gripper"]]
        analogSpeedGripper = diffGripper/analogUpdateTimes

        playtimeMsg = UInt8()
        playtimeMsg.data = self.bound(0,255,digitalPlaytime)
        self.speedsPub[self.servoIds["shoulder_pan"]].publish(playtimeMsg)
        self.speedsPub[self.servoIds["shoulder_lift"]].publish(playtimeMsg)
        self.speedsPub[self.servoIds["elbow"]].publish(playtimeMsg)

        speedMsg = Float32()

        speedMsg.data = analogSpeedWrist
        self.speedsPub[self.servoIds["wrist"]].publish(speedMsg)

        speedMsg.data = analogSpeedWristTwist
        self.speedsPub[self.servoIds["wrist_twist"]].publish(speedMsg)

        speedMsg.data = analogSpeedGripper
        self.speedsPub[self.servoIds["gripper"]].publish(speedMsg)

        positionMsg = UInt16()
        for i in range(len(self.positionPub)):
            positionMsg.data = self.bound(self.lowerSignalBound[i], self.upperSignalBound[i], self.transformAngleToSignal(state.position[i], i))
            self.positionPub[i].publish(positionMsg)
        
        self.lastState = state
        rospy.sleep(rospy.Duration(movementTime/100))

if __name__ == '__main__':
    rospy.init_node("arm_controller")
    controller = ArmController()
    rospy.spin()


