#!/usr/bin/env python
# BEGIN ALL
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage

from navigation import Robot
from scan import Scan
from transformations import Transformations

import tf
from tf.transformations import euler_from_quaternion


rospy.init_node('robot')

robot = Robot()
scan = Scan()
trans = Transformations()

final_dict = {}
qr_code = False
next_qr_pos = None
scan_output = None
final_dict = {}
robot_pos_dict = {}
qr_pos_hidden_dict = {}
obj_info_dict = {}
random = True
while not rospy.is_shutdown():
	while random is True:
		robot.random_search()
		scan_output = scan.scan()
		scan_val = scan.validate_scan()
		if scan_output is not None and scan_val == True:
			robot_pos = robot.get_current_position()
			robot.stop()
			qr_info, obj_info = scan_output
			print("Found QR Code")
			curr_qr_pos, next_qr_pos, num_qr, msg_qr = qr_info
			# Storing qr code positions in a dict
			qr_pos_hidden_dict[num_qr] = curr_qr_pos
			obj_info_dict[num_qr] = obj_info #position, orientation = obj_info
			final_dict[num_qr] = msg_qr
			robot_pos_dict[num_qr] = robot_pos
			scan_val = False
			if len(qr_pos_hidden_dict)==2:
				robot.stop()
				random = False
	
	trans.get_hidden_frame(robot_pos_dict, qr_pos_hidden_dict, obj_info_dict)
	break
	#tf_matrix_map_odom = trans.get_tf_transformation("map", "odom")
	#goal = np.zeros((4,1))
	#goal = np.zeros((3,1))
	#goal[0] = next_qr_pos[0]
	#goal[1] = next_qr_pos[1]
	#goal[2] = 0
	#goal[3] = 1
	#goal = np.matmul(tf_matrix_map_odom, goal)
	# tf_matrix_odom_bfoot = trans.get_tf_transformation("odom", "base_footprint")
	#tf_matrix_bfoot_blink = trans.get_tf_static_transformation("base_footprint", "base_link")
	#tf_matrix_cam_camo = trans.get_tf_static_transformation("camera_link", "camera_optical_link")
	#tf_matrix_imu_cam = trans.get_tf_static_transformation("imu_link", "camera_link")
	#tf_matrix_blink_imu = trans.get_tf_static_transformation("base_link", "imu_link")
	#goal = 0
	#robot.move_to_goal(goal)
	#rospy.sleep(30.)
	
	## transformations in tf_static:
	# base_footprint -> base_link
	# imu_link -> camera_link
	# camera_link -> camera_optical_link
	# base_link -> caster_back_link
	# base_link -> imu_link
	# base_link -> base_scan

	## transformation in tf:
	# map -> odom
	# odom -> base_footprint

	# TO-DO:
	# find transformation matrix for robot frame and map frame
	# calculate new goal for robot with next_qr_pos
	# implement motion planning for new goal



