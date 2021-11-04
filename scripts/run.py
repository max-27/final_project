#!/usr/bin/env python
# BEGIN ALL
import rospy
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage

from navigation import Robot
from scan import Scan
from tf.transformations import euler_from_quaternion


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

rospy.init_node('robot')

robot = Robot()
scan = Scan()
qr_code = False
next_qr_pos = None
scan_output = None
while not rospy.is_shutdown():
	while qr_code is False:
		robot.random_search()
		scan_output = scan.scan()
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
			#print(current_tf.transforms[0].transform.translation)
			#print(current_tf.transforms[0].transform.rotation)
			rotation_quat = current_tf.transforms[0].transform.rotation
			rotation_quat = [rotation_quat.x, rotation_quat.y, rotation_quat.z, rotation_quat.w]
			rotation_euler = euler_from_quaternion(rotation_quat)
			print(rotation_euler)
			break
	

	# find transformation matrix for robot frame and map frame
	# calculate new goal for robot with next_qr_pos
	# implement motion planning for new goal



