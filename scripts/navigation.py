#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Robot:

  def __init__(self):
    self.g_range_ahead = 1
    self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.driving_forward = True
    self.twist = Twist()
    rospy.init_node('navigation')


  def scan_callback(self, msg):
    #global g_range_ahead
    tmp=[msg.ranges[0]]
    for i in range(1,21):
      tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
      tmp.append(msg.ranges[i])
    self.g_range_ahead = min(tmp)


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


  def stop(self):
    print("Stop robot")
    self.twist.linear.x = 0.
    self.twist.linear.y = 0.
    self.twist.angular.z = 0.
    self.cmd_vel_pub.publish(self.twist)
