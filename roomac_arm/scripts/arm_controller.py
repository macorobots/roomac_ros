#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16, UInt8, Float32
import math

class ArmController:
    lastState = None
    changeThreshold = 0.01
    maxSpeed = 0.005 #rad/s
    maxMovementTime = 1000 #max value is 255, and it is rounded to it if excceding (in 10ms)
    analogUpdateDelay = 0.7 #real value 0.7 (in units of 10ms as playtime in digitalServo)
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
    analogSpeed = 2 #Change in ms in signal per analogUpdateDelay
    scalingFactors = [digitalScaleFactor, digitalScaleFactor, digitalScaleFactor/2., analogScaleFactor, analogScaleFactor, analogScaleFactor]
    
    def __init__(self):
        self.jointsSub = rospy.Subscriber("/joint_states", JointState, self.jointsStateCb)

        for i in range(len(self.jointNames)):
            self.positionPub.append(rospy.Publisher(self.jointNames[i]+"_position", UInt16, queue_size=5, latch=True))
        
        for i in range(3):
            self.speedsPub.append(rospy.Publisher(self.jointNames[i]+"_playtime", UInt8, queue_size=5, latch=True))
        
        for i in range(3, len(self.jointNames)):
            self.speedsPub.append(rospy.Publisher(self.jointNames[i]+"_speed", Float32, queue_size=5, latch=True))
        
        self.setDefaultPosition()
        
    def setDefaultPosition(self):
        playtimeMsg = UInt8()
        playtimeMsg.data = 255
        for digitalId in ["shoulder_pan", "shoulder_lift", "elbow"]:
            self.speedsPub[self.servoIds[digitalId]].publish(playtimeMsg)
        
        for i in range(len(self.positionPub)):
            positionMsg = UInt16()
            positionMsg.data = self.zeroPositions[i]
            self.positionPub[i].publish(positionMsg)



    def transformAngleToSignal(self, angle, id):
        return angle*self.scalingFactors[id] + self.zeroPositions[id]

    def bound(self, low, high, value):
        return max(low, min(high, value))

    def jointsStateCb(self, state):
        if not self.lastState:
            self.lastState = state
            return

        maxDigitalAngleDiff = 0
        for digitalId in ["shoulder_pan", "shoulder_lift", "elbow"]:
            positionDiff = abs(state.position[self.servoIds[digitalId]] - self.lastState.position[self.servoIds[digitalId]])
            if positionDiff > maxDigitalAngleDiff:
                maxDigitalAngleDiff = positionDiff

        maxAnalogAngleDiff = 0
        for analogId in ["wrist", "wrist_twist", "gripper"]:
            positionDiff = abs(state.position[self.servoIds[analogId]] - self.lastState.position[self.servoIds[analogId]])
            if positionDiff > maxAnalogAngleDiff:
                maxAnalogAngleDiff = positionDiff
        
        if maxDigitalAngleDiff < self.changeThreshold and maxAnalogAngleDiff < self.changeThreshold:
            return
        
        digitalMovementTime = min((maxDigitalAngleDiff/math.pi)*self.maxMovementTime, 255)
        analogMovementTime = ((maxAnalogAngleDiff*self.analogScaleFactor)/self.analogSpeed)*self.analogUpdateDelay
        
        if digitalMovementTime > analogMovementTime:
            movementTime = digitalMovementTime
        else:
            movementTime = analogMovementTime
        
        if movementTime > self.maxMovementTime:
            movementTime = self.maxMovementTime
        
        diffWrist = abs(state.position[self.servoIds["wrist"]] - self.lastState.position[self.servoIds["wrist"]])*self.scalingFactors[self.servoIds["wrist"]]

        diffWristTwist = abs(state.position[self.servoIds["wrist_twist"]] - self.lastState.position[self.servoIds["wrist_twist"]])*self.scalingFactors[self.servoIds["wrist_twist"]]

        diffGripper = abs(state.position[self.servoIds["gripper"]] - self.lastState.position[self.servoIds["gripper"]])*self.scalingFactors[self.servoIds["gripper"]]

        playtimeMsg = UInt8()
        playtimeMsg.data = self.bound(0,255,movementTime)
        for digitalId in ["shoulder_pan", "shoulder_lift", "elbow"]:
            self.speedsPub[self.servoIds[digitalId]].publish(playtimeMsg)

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


