#!/usr/bin/env python
# BEGIN ALL
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal



class Robot:

  def __init__(self):
    self.g_range_ahead = 1
    self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
    self.m_states_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
    self.driving_forward = True
    self.twist = Twist()
    self.goal = MoveBaseActionGoal()
    self.listener = tf.TransformListener()
    self.rate = rospy.Rate(1.0)
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
      self.twist.linear.x = 0.1
      self.twist.angular.z = 0.0
    else:
      #print("spin")
      self.twist.linear.x = 0.0
      self.twist.angular.z = 0.1
    self.cmd_vel_pub.publish(self.twist)

  def move_to_goal(self, goal_robot, frame_id, id):
    self.goal.goal_id.id = id
    #self.goal.goal_id.stamp =
    self.goal.goal.target_pose.header.frame_id = frame_id
    self.goal.goal.target_pose.pose.position.x = goal_robot[0]
    self.goal.goal.target_pose.pose.position.y = goal_robot[1]
    self.goal.goal.target_pose.pose.orientation.x = 0
    self.goal.goal.target_pose.pose.orientation.y = 0
    self.goal.goal.target_pose.pose.orientation.z = 0.321544014734
    self.goal.goal.target_pose.pose.orientation.w = 0.324784465306
    self.goal_pub.publish(self.goal)

  def stop(self):
    #print("Stop robot")
    self.twist.linear.x = 0.
    self.twist.linear.y = 0.
    self.twist.angular.z = 0.
    self.cmd_vel_pub.publish(self.twist)

  def get_current_position(self):
      while True:
          try:
              now = rospy.Time.now()
              self.listener.waitForTransform('/base_footprint', '/odom', now,rospy.Duration(5))
              (trans_footprint,rot_footprint) = self.listener.lookupTransform('/base_footprint', '/odom', now)
          except (tf.LookupException, tf.ConnectivityException):
              continue
          angles_footprint = euler_from_quaternion(rot_footprint)
          return [trans_footprint,angles_footprint]
