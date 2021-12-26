#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node("add_scene_objects")

  # Construct the initial scene object
  scene = moveit_commander.PlanningSceneInterface()
  
  # Create a scene publisher to push changes to the scene
  scene_pub = rospy.Publisher('planning_scene', moveit_commander.PlanningScene, queue_size=10)

  # Pause for the scene to get ready
  rospy.sleep(1)

  # Set the reference frame for pose targets
  reference_frame = 'base_up_1'
  robot_body_id = 'upper_body'

  # Remove leftover objects from a previous run
  scene.remove_world_object(robot_body_id)

  body_size = [0.21, 0.3, 0.5]

  body_pose = PoseStamped()
  body_pose.header.frame_id = reference_frame
  body_pose.pose.position.x = -body_size[0]/2 + (body_size[0]-0.15)/2 
  body_pose.pose.position.y = body_size[1]/2 - (body_size[1]-0.2)/2
  body_pose.pose.position.z = body_size[2]/2 - 0.1

  
  
  scene.add_box(robot_body_id, body_pose, body_size)

  rospy.sleep(1)

  moveit_commander.roscpp_shutdown()
  moveit_commander.os._exit(0)