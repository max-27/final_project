#!/usr/bin/env python
# BEGIN ALL
import rospy
import numpy as np
import pdb

from navigation import Robot
from scan import Scan
from transformations import Transformation

rospy.init_node('robot')

robot = Robot()
scan = Scan()
trans = Transformation()
<<<<<<< HEAD
scan_output = None
setting_new_goal = False
dict_global_qr_codes = {}
while not rospy.is_shutdown():
	robot.random_search()
	scan_output = scan.scan()
	# Finding first QR code
	while scan_output is not None and len(dict_global_qr_codes)<2:
		robot.stop()
		
		dict_global_qr_codes[scan_output[0][2]]=scan_output[0][0]
		print(dict_global_qr_codes)
		
		rospy.sleep(2)
		break
	while len(dict_global_qr_codes)==2:
		print('Two QR codes found.')
		robot.stop()
		print('Setting goal to third QR code.')
		robot.move_to_goal(scan_output[0][1], "qr_coordinate_frame")


	#while scan_output is not None:
	#	rospy.sleep(2) # Sleeps for 1 sec
	#	print('Sleeping over')
	#	#print('Setting goal to: {}'.format(scan_output[1]))
	#	print(scan_output[0][1])
	#	robot.move_to_goal(scan_output[0][1], "qr_coordinate_frame")
	#	#rans.get_qr_code()
	#	#success = trans.get_hidden_frame()
	







