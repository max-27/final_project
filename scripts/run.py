#!/usr/bin/env python
# BEGIN ALL
import rospy
import numpy as np

from navigation import Robot
from scan import Scan
from transformations import Transformation

rospy.init_node('robot')

robot = Robot()
scan = Scan()
trans = Transformation()
scan_output = None
while not rospy.is_shutdown():
	while scan_output is None:
		robot.random_search()
		scan_output = scan.scan()
	robot.stop()
	trans.get_qr_code()
	success = trans.get_hidden_frame()
	if success:
		break
		






