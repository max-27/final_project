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

bool_random_search = True
scan_output = None
setting_target_goal = False
dict_global_qr_codes = {}
while not rospy.is_shutdown():
	
	## Start-phase: Robot should find two QR codes with random search

	while bool_random_search is True:
		if len(dict_global_qr_codes)<2:
			robot.random_search()
			scan_output = scan.scan()

			if scan_output is not None:
				# return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
				current_qr_numb = scan_output[0][2]
				current_qr_locat = scan_output[0][0]
				next_qr_locat = scan_output[0][1]

				dict_global_qr_codes[current_qr_numb]=[current_qr_locat,next_qr_locat]
				print(dict_global_qr_codes)
		else:
			robot.stop()
			print('Two QR codes found.')
			bool_random_search = False
			break
			
	while bool_random_search is False:
		
		if current_qr_numb+1 not in dict_global_qr_codes.keys():
			goal = np.array(next_qr_locat)-np.array([0.5,0.5])
			robot.move_to_goal(goal, "qr_coordinate_frame")

#
#	while setting_target_goal is False and len(dict_global_qr_codes)==2:
#		print('Two QR codes found.')
#		robot.stop()
#		print('Setting goal to third QR code.')
#		goal = np.array(scan_output[0][1])-np.array([1.5,1.5])
#		setting_target_goal = True
#		break
#
#	while setting_target_goal is True:
#		
#		#pdb.set_trace()
#		robot.move_to_goal(goal, "qr_coordinate_frame")


	#while scan_output is not None:
	#	rospy.sleep(2) # Sleeps for 1 sec
	#	print('Sleeping over')
	#	#print('Setting goal to: {}'.format(scan_output[1]))
	#	print(scan_output[0][1])
	#	robot.move_to_goal(scan_output[0][1], "qr_coordinate_frame")
	#	#rans.get_qr_code()
	#	#success = trans.get_hidden_frame()
																																																																																																																																			







