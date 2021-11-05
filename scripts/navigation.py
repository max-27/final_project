#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates


class Robot:

  def __init__(self):
    self.g_range_ahead = 1
    self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
    self.m_states_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.driving_forward = True
    self.twist = Twist()
    #rospy.init_node('navigation')


  def scan_callback(self, msg):
    #global g_range_ahead
    tmp=[msg.ranges[0]]
    for i in range(1,21):
      tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
      tmp.append(msg.ranges[i])
    self.g_range_ahead = min(tmp)

  def amcl_callback(self, msg):
    robot_pos = msg.pose.pose.position
    self.robot_position = [robot_pos.x, robot_pos.y, robot_pos.z]
    self.robot_orientation = msg.pose.pose.orientation

  def random_search(self):
    if self.g_range_ahead < 0.8:
      self.driving_forward = False
    else: # we're not driving_forward
      self.driving_forward = True # we're done spinning, time to go forward!

    if self.driving_forward:
      #print("drive forward")
      self.twist.linear.x = 0.4
      self.twist.angular.z = 0.0
    else:
      #print("spin")
      self.twist.linear.x = 0.0
      self.twist.angular.z = 0.4
    self.cmd_vel_pub.publish(self.twist)

  def move_to_goal(self, goal):
    while abs(self.robot_position[0] - goal[0]) <= 0.0001 and abs(self.robot_position[1] - goal[1]) <= 0.0001:


  def stop(self):
    print("Stop robot")
    self.twist.linear.x = 0.
    self.twist.linear.y = 0.
    self.twist.angular.z = 0.
    self.cmd_vel_pub.publish(self.twist)
