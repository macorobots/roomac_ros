#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy
import numpy as np


class Painter:
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.move_group =  moveit_commander.MoveGroupCommander("right_arm")

    # If there are problems with finding plan, although it didn't look to work properly
    # (actual number of attempts was different)
    self.move_group.allow_replanning(True)
    self.move_group.set_planning_time(60)
    self.move_group.set_num_planning_attempts(10)

    # only position ik, really important without it fails to find plan
    self.move_group.set_goal_orientation_tolerance(10.)

    self.move_group.set_goal_position_tolerance(0.001)

    self.retraction = 0.05

  
  def go_to_point(self, point):
    self.move_group.set_start_state_to_current_state()

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position = point

    self.move_group.set_pose_target(pose_goal)

    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()

  def paint_point(self, point):
    pre_point = copy.deepcopy(point)
    pre_point.y -= self.retraction

    self.go_to_point(pre_point)
    self.go_to_point(point)    
    self.go_to_point(pre_point)    

  def paint_corners(self):
    width = 0.2
    height = 0.2
    
    pt1 = geometry_msgs.msg.Point()

    pt1.x = 0.03
    pt1.y = 0
    pt1.z = 0.75

    for i in np.arange(0, width, 0.02):
      for j in np.arange(0,height, 0.02):
        pt = copy.deepcopy(pt1)
        pt.x += i
        pt.z += j

        self.paint_point(pt)

if __name__ == "__main__":
  
  rospy.init_node("go_to_goal_tutorial")

  painter = Painter()
  painter.paint_corners()