#!/usr/bin/env python
# BEGIN ALL
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray


class Robot:

    def __init__(self):
        self.g_range_ahead = 1
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.m_states_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        self.pub_cancel_goal = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        self.pub_goal_simple = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1.)
        self.sub_status = rospy.Subscriber("move_base/status", GoalStatusArray, self._goal_status_callback)
        self.driving_forward = True
        self.twist = Twist()
        self.goal = MoveBaseActionGoal()
        self.cancel_msg = GoalID()
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1.0)
        self.simple_goal = PoseStamped()
        self.checkpoint = 0
        self.nextCheckPoint = [-5.5, 2.]
        self.running = False

    def _goal_status_callback(self, msg):
        if len(msg.status_list) > 0:
            self.status_goal = msg.status_list[0].status
        else:
            self.status_goal = ''

    def scan_callback(self, msg):
        # global g_range_ahead
        tmp = [msg.ranges[0]]
        for i in range(1, 21):
            tmp.append(msg.ranges[i])
        for i in range(len(msg.ranges) - 21, len(msg.ranges)):
            tmp.append(msg.ranges[i])
        self.g_range_ahead = min(tmp)

    def amcl_callback(self, msg):
        robot_pos = msg.pose.pose.position
        self.robot_position = [robot_pos.x, robot_pos.y, robot_pos.z]
        self.robot_orientation = msg.pose.pose.orientation

    def random_search(self):
        if self.g_range_ahead < 0.8:
            self.driving_forward = False
        else:  # we're not driving_forward
            self.driving_forward = True  # we're done spinning, time to go forward!

        if self.driving_forward:
            # print("drive forward")
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
        else:
            # print("spin")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.12
        self.cmd_vel_pub.publish(self.twist)

    def move_to_goal(self, goal_robot, frame_id, id_):
        self.goal.goal_id.id = id_
        self.goal.goal.target_pose.header.frame_id = frame_id
        self.goal.goal.target_pose.pose.position.x = goal_robot[0]
        self.goal.goal.target_pose.pose.position.y = goal_robot[1]
        self.goal.goal.target_pose.pose.orientation.x = 0
        self.goal.goal.target_pose.pose.orientation.y = 0
        self.goal.goal.target_pose.pose.orientation.z = 0.321544014734
        self.goal.goal.target_pose.pose.orientation.w = 0.324784465306
        self.goal_pub.publish(self.goal)

    def cancel_goal(self, goal_id):
        self.cancel_msg.id = goal_id
        self.cancel_pub.publish(self.cancel_msg)

    def move_forward(self):
        self.twist.linear.x = 0.02
        self.cmd_vel_pub.publish(self.twist)

    def spin_360(self, spinning_speed=0.05):
        self.twist.angular.z = spinning_speed
        self.cmd_vel_pub.publish(self.twist)

    def stop(self):
        # print("Stop robot")
        self.twist.linear.x = 0.
        self.twist.linear.y = 0.
        self.twist.angular.z = 0.
        self.cmd_vel_pub.publish(self.twist)

    def focused_search(self):

        if self.running == False:
            rospy.sleep(2.)
            print('Robot receives coordinates to:{}'.format(self.nextCheckPoint))

            self.simple_move_to_goal(self.nextCheckPoint, frame_id="map")

            rospy.sleep(3.)

            self.running = True


        elif self.status_goal == 1 and self.running is True:
            print("Running")

        elif self.status_goal == 2 and self.running is True:
            print('Goal canceled. Robot starts turning.')


        elif self.status_goal == 3 and self.running is True:
            print("Checkpoint reached")

            # rospy.sleep(30.)

            if self.checkpoint == 1:
                print('Status Checkpoint 1')
                self.nextCheckPoint[1] = -1 * self.nextCheckPoint[1]

            elif self.checkpoint == 2:
                print('Status Checkpoint 2')
                self.nextCheckPoint[0] = -1 * self.nextCheckPoint[0]

            elif self.checkpoint == 3:
                print('Status Checkpoint 3')
                self.nextCheckPoint[1] = -1 * self.nextCheckPoint[1]

            elif self.checkpoint == 4:
                print('Status Checkpoint 4')
                print("Expanding search")
                self.nextCheckPoint[0] = abs(self.nextCheckPoint[0]) - 1
                self.nextCheckPoint[1] = abs(self.nextCheckPoint[1]) - 0.5
                self.checkpoint = 1
            self.checkpoint = self.checkpoint + 1
            rospy.sleep(2.)
            self.running = False
            self.canceling_goal()


        elif self.status_goal == 4 and self.running is True:
            print("Goal not reachable, going to next one")
            self.status_goal = 3
            self.running = False

    def move_to_checkpoint(self, frame_id='map'):
        rospy.sleep(3.)
        self.simple_goal.header.frame_id = frame_id
        self.simple_goal.pose.position.x = self.nextCheckPoint[0]
        self.simple_goal.pose.position.y = self.nextCheckPoint[1]
        self.simple_goal.pose.orientation.w = 1.
        print('Published goal:{}'.format(self.simple_goal))
        self.pub_goal_simple.publish(self.simple_goal)
        print('Inside function move_first checkpoint, status: {}'.format(self.goal))
        rospy.sleep(3.)

    def simple_move_to_goal(self, goal_coordinates, frame_id='map'):

        self.simple_goal.header.frame_id = frame_id
        self.simple_goal.pose.position.x = goal_coordinates[0]
        self.simple_goal.pose.position.y = goal_coordinates[1]
        self.simple_goal.pose.orientation.w = 1.0
        #print('Published goal:{}'.format(self.simple_goal))

        # print('Goal status: {}'.format(self.status_id))
        self.pub_goal_simple.publish(self.simple_goal)

    def canceling_goal(self):
        # self.cancel_goal.id = str(self.checkpoint)
        self.pub_cancel_goal.publish(self.cancel_goal)

    def turn(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.05
        self.cmd_vel_pub.publish(self.twist)

    def get_current_position(self):
        while True:
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform('/base_footprint', '/odom', now, rospy.Duration(5))
                (trans_footprint, rot_footprint) = self.listener.lookupTransform('/base_footprint', '/odom', now)
            except (tf.LookupException, tf.ConnectivityException):
                continue
            angles_footprint = euler_from_quaternion(rot_footprint)
            return [trans_footprint, angles_footprint]
