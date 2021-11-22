#!/usr/bin/env python
# BEGIN ALL
import rospy
import tf
import time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult




class Robot:

  def __init__(self):
    self.g_range_ahead = 1
    self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
    self.m_states_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
    rospy.Subscriber("move_base/status", GoalStatusArray, callback_status)
    self.driving_forward = True
    self.twist = Twist()
    self.goal = MoveBaseActionGoal()
    self.listener = tf.TransformListener()
    self.rate = rospy.Rate(1.0)
    self.stuck_counter = 0
    self.checkpoint = 0
    self.nextCheckPoint = [1,0.5]
    self.running = False
    self.oldPosition = [0,0]
    self.status_id = 0
    self.status_goal = 0
    #rospy.init_node('navigation')
    time.sleep(2) #needs time to properly subscribe. 

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
    robot_ori = msg.pose.pose.orientation
    self.robot_position = [robot_pos.x, robot_pos.y, robot_pos.z]
    self.robot_orientation = robot_ori.z
    print("pose updated")

  def callback_status(msg):
    if len(msg.status_list) > 0:
        self.status_goal = msg.status_list[0].status
        self.status_id = msg.status_list[0].goal_id.id


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

  def move_to_goal(self, goal_robot, frame_id,id_):
    self.goal.goal.target_pose.header.frame_id = frame_id
    self.goal.goal.target_pose.pose.position.x = goal_robot[0]
    self.goal.goal.target_pose.pose.position.y = goal_robot[1]
    self.goal.goal.target_pose.pose.orientation.x = 0
    self.goal.goal.target_pose.pose.orientation.y = 0
    self.goal.goal.target_pose.pose.orientation.z = 0.321544014734
    self.goal.goal.target_pose.pose.orientation.w = 0.324784465306
    self.goal_pub.publish(self.goal)
    print("Published goal")

  def focused_search2(self):
    
    if self.status_id == 4:
      print("Goal not reachable, going to next one")
      self.checkpoint = self.checkpoint +1

    elif self.status_id == 3:
      print("checkpoint reached")
      self.checkpoint = self.checkpoint + 1
      #self.stop2()
      self.running = False
      if self.checkpoint == 1:
        self.nextCheckPoint[1] = -1*self.nextCheckPoint[1]
      elif self.checkpoint == 2:
        self.nextCheckPoint[0] = -1*self.nextCheckPoint[0]
      elif self.checkpoint == 3:
        self.nextCheckPoint[1] = -1*self.nextCheckPoint[1]
      elif self.checkpoint == 4:
        print("Expanding search")
        self.nextCheckPoint[0] = self.nextCheckPoint[0] + 1
        self.nextCheckPoint[1] = self.nextCheckPoint[1] + 0.5
      else:
        print("Search failed, starting over")
        self.checkpoint = 1
        self.nextCheckPoint[0] = 1
        self.nextCheckPoint[1] = 0.5
        self.moveResult = "Nothing"

    elif self.running == False:
      self.move_to_goal(self.nextCheckPoint, "map",self.checkpoint)
      self.running = True

    else:
      print("Running")
      time.sleep(2) #to not flood the terminal



  def focused_search(self):
    left=0.75
    right=-0.72
    front=0.0
    back=0.5

    if self.checkpoint == 0:
      y = 2
      x = 0 #going for y-border. 
      direction = 0.70 #old Left
    if self.checkpoint == 1:
      y = 0 # going for x-border.
      x = 5
      direction = 0.97 #old Back
    else:
      print("Search complete")
      print(self.checkpoint)



    if self.g_range_ahead > 0.6:
      if self.robot_orientation > direction-0.04 and self.robot_orientation < direction+0.04:
        self.stop()
        print("Orientation reached")

      else:
        if self.robot_orientation > left:
          self.turn_right()
        else:
          self.turn_left()
      if (self.robot_position[1] > y and x == 0) or (self.robot_position[0] > x and y == 0): 
        self.stop()
        print("Border reached")
        self.checkpoint = self.checkpoint + 1
        self.stop()
        self.turn() #turn away from the border
        time.sleep(2) #give it time to prepare for next checkpoint. 
      else:
        self.stuck_counter = 0
        print(self.robot_position[1],y, self.checkpoint)
        self.straight()
    
    else:
      self.stop()
      print("Object in front")
      if self.stuck_counter < 30:
        self.turn_left()
        self.stuck_counter = self.stuck_counter + 1
        time.sleep(1)
      else:
        if self.stuck_counter > 100:
          print("HELP!")
          self.stop()
        else:
          self.turn_right()
          self.stuck_counter = self.stuck_counter + 1
          time.sleep(1)
      
     
  def stop(self):
    print("Stop robot")
    self.twist.linear.x = 0.
    self.twist.linear.y = 0.
    self.twist.angular.z = 0.
    self.cmd_vel_pub.publish(self.twist)

  def stop2(self):
    self.cancel_goal(self.checkpoint)

  def turn(self):
    self.twist.angular.z = 0.4
    self.cmd_vel_pub.publish(self.twist)

  def turn_left(self):
    print("Turning left")
    self.twist.angular.z = 0.2
    self.cmd_vel_pub.publish(self.twist)

  def turn_right(self):
    print("Turning right")
    self.twist.angular.z = -0.2
    self.cmd_vel_pub.publish(self.twist)
  
  def straight(self):  
    print("Going straight") 
    self.twist.linear.x = 0.2
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
