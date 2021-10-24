#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoyRobotController:
    
    def __init__(self):
        self.joySub = rospy.Subscriber("/joy", Joy, self.joyCb)
        self.jointsPub = rospy.Publisher("/joy_joint_states", JointState, queue_size=5)
        self.ttsPub = rospy.Publisher("/tts", String, queue_size=5)


        self.zeroPosition = [0.0, 0.2, 0.6, 0.0, 0.0, 0.0, 0.0]
        self.midPosition = [-0.6562571023320758, 0.9862048106645759, 1.5288, 0.0, 0.7577515079999997, 0.40787999999999996, 0.40787999999999996]
        self.highPosition = [-2.44979083333, 0.5, 1.5, 0.0, -0.2, 0.6, 0.6]
        self.midPosition2 = [-0.529834284748889, 0.695633915581579, 0.58968, 0.0, -1.486914547, 0.372, 0.372]

        self.jointMsg = JointState()

        self.jointNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint', 'wrist_twist_joint', 'left_gripper_joint', 'right_gripper_joint']

    def joyCb(self, msg):
        #4 5 6 7
        jointMsg = JointState()
        jointMsg.name = self.jointNames
        if msg.buttons[4]:
            jointMsg.position = self.highPosition
            self.jointsPub.publish(jointMsg)
        elif msg.buttons[5]:
            jointMsg.position = self.midPosition
            self.jointsPub.publish(jointMsg)
        elif msg.buttons[6]:
            jointMsg.position = self.zeroPosition
            self.jointsPub.publish(jointMsg)
        elif msg.buttons[7]:
            jointMsg.position = self.midPosition2
            self.jointsPub.publish(jointMsg)
        elif msg.buttons[10]:
            txt = String()
            txt.data = "Witam"
            self.ttsPub.publish(txt)



if __name__ == '__main__':
    rospy.init_node("joy_robot_controller")
    controller = JoyRobotController()
    rospy.spin()


