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

	tf_matrix_map_odom = trans.get_tf_transformation("map", "odom")
	print(1)
	tf_matrix_odom_bfoot = trans.get_tf_transformation("odom", "base_footprint")
	print(2)
	tf_matrix_bfoot_blink = trans.get_tf_static_transformation("base_footprint", "base_link")
	#tf_matrix_cam_camo = trans.get_tf_static_transformation("camera_link", "camera_optical_link")
	print(3)
	tf_matrix_imu_cam = trans.get_tf_static_transformation("imu_link", "camera_link")
	tf_matrix_blink_imu = trans.get_tf_static_transformation("base_link", "imu_link")
	

	break
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



