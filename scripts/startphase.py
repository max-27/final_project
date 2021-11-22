#!/usr/bin/env python
# BEGIN ALL
import rospy
import numpy as np
import pdb
import tf

import pdb
from navigation import Robot
from scan import Scan
from transformations import Transformation
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped



rospy.init_node('robot')

robot = Robot()
scan = Scan()
transform = Transformation()

bool_random_search = True
scan_output = None
setting_target_goal = False
intermediate_step = False

dict_global_qr_codes = {}
dict_hidden_qr_codes = {}
intermediate_goal = []
listener = tf.TransformListener()


def callback_status(msg):
    global status_goal
    global status_id
    if len(msg.status_list) > 0:
        status_goal = msg.status_list[0].status
        status_id = msg.status_list[0].goal_id.id



rospy.init_node('startphase')


sub_status = rospy.Subscriber("move_base/status", GoalStatusArray, callback_status)
pub_goal = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1.)
pub_goal_cancel = rospy.Publisher('move_base/cancel',PoseStamped,queue_size=1.)

posestamp = PoseStamped()


while not rospy.is_shutdown():
    posestamp.pose.position = [0.,0.,0.] #Origin
    posestamp.pose.orientation = [0.,0.,1.,1.]


    #Move to Origin of map
    pub_goal.publish(posestamp)

