#!/usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, Twist, Vector3, Pose2D, QuaternionStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

import math
import copy

class ImuOrientationRepublisher:
  def __init__(self):
    self.imuPub = rospy.Publisher("/imu/orientation", Imu, queue_size=50)
    self.imuRawSub = rospy.Subscriber("/imu/orientation_raw", QuaternionStamped, self.imuCb)

    self.imuMsg = Imu()
    self.imuMsg.header.frame_id = "imu"

    # Disable, as in msg description
    self.imuMsg.angular_velocity_covariance[0] = -1

    # Disable, as in msg description
    self.imuMsg.linear_acceleration_covariance[0] = -1

  def imuCb(self, msg):
    self.imuMsg.orientation = msg.quaternion
    self.imuMsg.header.stamp = msg.header.stamp
    self.imuPub.publish(self.imuMsg)

class ImuAngularVelocityRepublisher:
  def __init__(self):
    self.imuPub = rospy.Publisher("/imu/angular_velocity", Imu, queue_size=50)
    self.imuRawSub = rospy.Subscriber("/imu/angular_velocity_raw", Vector3Stamped, self.imuCb)

    self.imuMsg = Imu()
    self.imuMsg.header.frame_id = "imu"

    # Disable, as in msg description
    self.imuMsg.orientation_covariance[0] = -1

    # Disable, as in msg description
    self.imuMsg.linear_acceleration_covariance[0] = -1

  def imuCb(self, msg):
    self.imuMsg.angular_velocity = msg.vector
    self.imuMsg.header.stamp = msg.header.stamp
    self.imuPub.publish(self.imuMsg)

class OdomRepublisher:
  def __init__(self):
    self.odomPub = rospy.Publisher("/wheel_odom/odom", Odometry, queue_size=50)

    self.lastTime = rospy.Time.now()
    self.lastPosition = Pose2D()

    self.odomRawSub = rospy.Subscriber("/wheel_odom/position_raw", PoseStamped, self.odomCb)

  def odomCb(self, msg):
    currentTime = msg.header.stamp

    currentPosition = Pose2D()

    currentPosition.x = msg.pose.position.x
    currentPosition.y = msg.pose.position.y
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    eulerAngles = euler_from_quaternion(quaternion)
    currentPosition.theta = eulerAngles[2]

    dt = (currentTime - self.lastTime).to_sec()
    if dt > 0:
        vx = math.sqrt((currentPosition.x - self.lastPosition.x)**2 + (currentPosition.y - self.lastPosition.y)**2)/dt
        
        # Calculate direction of vx
        # atan2 The returned value is between PI and -PI.
        angle = math.atan2((currentPosition.y - self.lastPosition.y), (currentPosition.x - self.lastPosition.x))
        directionAngleDiff = abs(angle - self.lastPosition.theta)

        if directionAngleDiff > math.pi/2:
            vx = -vx

        angleDiff = math.fmod((currentPosition.theta - self.lastPosition.theta), 2*math.pi)
        if angleDiff > math.pi:
          angleDiff = 2*math.pi - angleDiff
        elif angleDiff < -math.pi:
          angleDiff = 2*math.pi + angleDiff
          
        vth = angleDiff/dt
    else:
        vx = 0
        vth = 0

    self.lastPosition = copy.deepcopy(currentPosition)

    odomMsg = Odometry()
    odomMsg.header.stamp = currentTime
    odomMsg.header.frame_id = "odom"
    odomMsg.child_frame_id = "base_link"
    odomMsg.pose.pose = Pose(Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    odomMsg.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
    
    self.odomPub.publish(odomMsg)

    self.lastTime = currentTime


if __name__ == '__main__':
    rospy.init_node('odom_imu_republisher')
    odomRepub = OdomRepublisher()
    imuOrientRepub = ImuOrientationRepublisher()
    imuAngVelRebup = ImuAngularVelocityRepublisher()
    rospy.spin()