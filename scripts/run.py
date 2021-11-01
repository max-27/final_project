#!/usr/bin/env python
# BEGIN ALL
import rospy
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage

from navigation import Robot
from scan import scan


currentModelState = ModelStates() 

# Definition of callback functions

def ModelStateCallback(msg):
  global currentModelState
  currentModelState = msg

def TFCallback(msg):
	global current_tf
	current_tf = msg

final_dict = {}
pose = rospy.Subscriber('/gazebo/model_states', ModelStates, ModelStateCallback)

tf = rospy.Subscriber('/tf', TFMessage, TFCallback)



robot = Robot()
qr_code = False
next_qr_pos = None
while not rospy.is_shutdown():
	while qr_code is False:
		robot.random_search()
		rospy.sleep(3.)
		scan_output = scan()
		if scan_output is not None:
			qr_info, obj_info = scan_output
			print("Found QR Code")
			qr_info, obj_info = scan_output
			curr_qr_pos, next_qr_pos, num_qr, msg_qr = qr_info
			final_dict[num_qr] = msg_qr
			position, orientation = obj_info
			robot.stop()
			qr_code = True

	#print(type(current_tf.transforms[0].child_frame_id))
	if current_tf.transforms[0].header.frame_id == "map" and current_tf.transforms[0].child_frame_id == "odom":
			print(current_tf.transforms[0].transform.translation)
			print(current_tf.transforms[0].transform.rotation)
			break
	

	# find transformation matrix for robot frame and map frame
	# calculate new goal for robot with next_qr_pos
	# implement motion planning for new goal



