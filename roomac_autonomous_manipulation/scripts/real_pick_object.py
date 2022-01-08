#!/usr/bin/env python

import sys
import copy

import rospy
import moveit_commander

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf
from tf.transformations import quaternion_from_euler

class Grasp:
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.move_group =  moveit_commander.MoveGroupCommander("right_arm")
    self.scene = moveit_commander.PlanningSceneInterface()

    # If there are problems with finding plan, although it didn't look to work properly
    # (actual number of attempts was different)
    self.move_group.allow_replanning(True)
    self.move_group.set_planning_time(60)
    self.move_group.set_num_planning_attempts(10)

    # only position ik, really important without it fails to find plan
    self.move_group.set_goal_orientation_tolerance(0.1)

    self.move_group.set_goal_position_tolerance(0.01)

    self.retraction = 0.1

    self.controller_command_pub = rospy.Publisher("/roomac/arm_position_controller/command", JointTrajectory, queue_size=10, latch=True)
    self.gripper_command = rospy.Publisher("/joint_states", JointState, queue_size=5)


  
  def go_to_point(self, point):
    self.move_group.set_start_state_to_current_state()

    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(-1.461, -1.445, 3.016)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]

    pose_goal.position = point

    # self.move_group.set_pose_target(pose_goal)
    
    # Allow approximate solutions (True)
    self.move_group.set_joint_value_target(pose_goal, "gripper_right_grip", True)

    plan = False
    while not plan:
      rospy.loginfo("Trying to find and execute plan")
      plan = self.move_group.go(wait=True)
    
    self.move_group.stop()
    self.move_group.clear_pose_targets()

  def move_gripper(self, position):
    gripper_msg = JointState()
    gripper_msg.name = ["right_gripper"]
    gripper_msg.position = [position]
    self.gripper_command.publish(gripper_msg)

    rospy.Rate(1.0).sleep()
  
  def close_gripper(self):
    rospy.loginfo("Sending close gripper command")
    self.move_gripper(0.1)
  
  def open_gripper(self):
    rospy.loginfo("Sending open gripper command")
    # Almost maximum, if needed can go to 1.2
    self.move_gripper(1.0)


  def grasp(self, point):
    reference_frame = 'base_link'
    object_id = 'object'

    # Remove leftover objects from a previous run
    self.scene.remove_world_object(object_id)

    body_size = [0.005, 0.005, 0.1]

    body_pose = PoseStamped()
    body_pose.header.frame_id = reference_frame
    body_pose.pose.position.x = point.x
    body_pose.pose.position.y = point.y
    body_pose.pose.position.z = point.z

    self.scene.add_box(object_id, body_pose, body_size)

    pre_point = copy.deepcopy(point)
    pre_point.y -= self.retraction

    post_point = copy.deepcopy(point)
    post_point.z += 0.2

    self.open_gripper()
    self.go_to_point(pre_point)
    self.go_to_point(point)
    self.close_gripper()
    self.go_to_point(post_point)

if __name__ == "__main__":
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node("go_to_goal_tutorial")
  grasp = Grasp()

  listener = tf.TransformListener()

  rate = rospy.Rate(10.0)
  tf_ready = False
  trans = None
  while not rospy.is_shutdown() and not tf_ready:
      try:
          (trans,rot) = listener.lookupTransform('base_link', 'ar_marker_2', rospy.Time(0))
          tf_ready = True
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      rate.sleep()

  pt = geometry_msgs.msg.Point()
  pt.x = trans[0]+0.04-0.02
  pt.y = trans[1]-0.09+0.01
  pt.z = trans[2]+0.05+0.05

  grasp.grasp(pt)