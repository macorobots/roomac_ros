#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg


if __name__ == "__main__":
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node("go_to_goal_tutorial")

  move_group =  moveit_commander.MoveGroupCommander("right_arm")


  # If there are problems with finding plan, although it didn't look to work properly
  # (actual number of attempts was different)
  move_group.allow_replanning(True)
  move_group.set_planning_time(60)
  move_group.set_num_planning_attempts(10)

  # Doesn't work
  move_group.set_start_state_to_current_state()

  # only position ik, really important without it fails to find plan
  move_group.set_goal_orientation_tolerance(10.)

  move_group.set_goal_position_tolerance(0.01)

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.w = 1.0
  pose_goal.position.x = 0.138
  pose_goal.position.y = -0.020
  pose_goal.position.z = 0.663

  move_group.set_pose_target(pose_goal)

  plan = move_group.go(wait=True)

  # Alternative
  # plan = move_group.plan()
  # if plan.joint_trajectory.points:  # True if trajectory contains points
  #     move_success = move_group.execute(plan)
  # else:
  #     rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

  # Calling `stop()` ensures that there is no residual movement
  move_group.stop()

  # It is always good to clear your targets after planning with poses.
  # Note: there is no equivalent function for clear_joint_value_targets()
  move_group.clear_pose_targets()