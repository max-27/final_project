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

sub_object_msg = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, self._scan_qr_pos_callback)



# Callbacks for subscribers
def _scan_qr_message_callback(self, msg):
        # message looks like this:
        # <data: "X=2.67\r\nY=3.23\r\nX_next=0.1\r\nY_next=3.5\r\nN=2\r\nL=M">
        self.code_message = msg.data

def _goal_status_callback(self, msg):
        self.goal_status = msg.status_list.status        

# Variable initialization
posestamp = PoseStamped()
self.checkpoint = 0
self.nextCheckPoint = [1,0.5]
self.running = False

def focused_search(self):
    
    if self.goal_status == 4:
      print("Goal not reachable, going to next one")
      self.goal_status = 3

    elif self.goal_status == 3:
        self.goal_status = 0
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


while not rospy.is_shutdown():
    posestamp.pose.position = [0.,0.,0.] #Origin
    posestamp.pose.orientation = [0.,0.,1.,1.]


    #Move to Origin of map
    pub_goal.publish(posestamp)

