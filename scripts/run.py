#!/usr/bin/env python
# BEGIN ALL
import rospy
from gazebo_msgs.msg import ModelStates

from navigation import Robot
from scan import scan

currentModelState = ModelStates() 


# Definition of callback functions

def ModelStateCallback(msg):
  global currentModelState
  currentModelState = msg


final_dict = {}
pose = rospy.Subscriber('/gazebo/model_states', ModelStates, ModelStateCallback)

robot = Robot()
qr_code = False
while not rospy.is_shutdown():
	while qr_code is False:
		robot.random_search()
		scan_output = scan()
		if  scan_output is not None:
			print("Found QR Code")
			qr_info, obj_info= scan_output
			curr_qr_pos, next_qr_pos, num_qr, msg_qr = qr_info
			final_dict[num_qr] = msg_qr
			position, orientation = obj_info
			robot.stop()
			qr_code = True
			robot_idx = currentModelState.name.index("turtlebot3_burger")


	print(curr_qr_pos)
	print(next_qr_pos)
	print(position)
	print(currentModelState.pose[robot_idx])
	break
	# find transformation matrix for robot frame and map frame
	# calculate new goal for robot with next_qr_pos
	# implement motion planning for new goal



