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
scan_output_1 = None
scan_output_2 = None
while not rospy.is_shutdown():
	while scan_output_1 is None:
		print("start random search for qr code 1")
		robot.random_search()
		scan_output_1 = scan.scan()
		val = scan.validate_scan()

	robot.stop()
	trans.get_qr_code()
	success = trans.get_hidden_frame()
	print("success {}".format(success))
	pdb.set_trace()

	if success:
		val = False
		while scan_output_2 is None and val is False:
			print("start random search for qr code 2")
			robot.random_search()
			scan_output_2 = scan.scan()
			val = scan.validate_scan()
		robot.stop()
		trans.get_qr_code()
		success = trans.get_hidden_frame()
		print("success{}".format(success))

	if success:
		break







