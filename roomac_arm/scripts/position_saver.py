#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Empty
import yaml
import io
import rospkg

class SequenceSaver:
    states = []
    lastState = None
    
    def __init__(self):
        self.jointsSub = rospy.Subscriber("/joint_states", JointState, self.jointsStateCb)
        self.sequenceSub = rospy.Subscriber("/sequence_command", Int16, self.sequenceCb)
        self.savePositionSub = rospy.Subscriber("/save_position", Empty, self.savePositionCb)
        self.clearCommandsSub = rospy.Subscriber("/clear_saved_positions", Empty, self.clearCommandsCb)

        self.jointsPub = rospy.Publisher("/sequence_joint_states", JointState, queue_size=5)

        self.rospack = rospkg.RosPack()

        # with open(self.rospack.get_path('roomac_arm')+"/scripts/positions_saved.yaml", 'r') as stream:
        #     data_loaded = yaml.safe_load(stream)
        #     rospy.logwarn("Positions loaded" + str(data_loaded))
    
    # def savePositionsToFile(self):
    #     with io.open(self.rospack.get_path('roomac_arm')+"/scripts/positions_saved.yaml", 'w', encoding='utf8') as outfile:
    #         yaml.dump((self.states), outfile, default_flow_style=False, allow_unicode=True)
        

    def jointsStateCb(self, state):
        self.lastState = state
    
    def savePositionCb(self, msg):
        self.states.append(self.lastState)
    
    def clearCommandsCb(self, msg):
        self.states = []
    
    def sequenceCb(self, command):
        if command.data == -1:
            for x in self.states:
                self.jointsPub.publish(x)
                rospy.sleep(rospy.Duration(1.))
        else:
            if command.data >= 0 and command.data < len(self.states):
                self.jointsPub.publish(self.states[command.data])

        

if __name__ == '__main__':
    rospy.init_node("position_saver")
    saver = SequenceSaver()
    # rospy.on_shutdown(saver.savePositionsToFile)
    rospy.spin()


