#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def scan_qr_pos_callback(msg):
	global next_qr_pos 
	next_qr_pos = msg.pose


# message looks like this:
# data: "X=2.67\r\nY=3.23\r\nX_next=0.1\r\nY_next=3.5\r\nN=2\r\nL=M"
def scan_qr_message_callback(msg):
	global qr_message
	qr_message = msg.data
  

def scan():
	# check if /visp_auto_tracker/object_position topic is published
	rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, scan_qr_pos_callback)
	rospy.Subscriber('/visp_auto_tracker/code_message', String, scan_qr_message_callback)

	next_qr_pos = rospy.wait_for_message("/visp_auto_tracker/object_position", PoseStamped)
	qr_message = rospy.wait_for_message('/visp_auto_tracker/code_message', String)

	if next_qr_pos is not None:
		print(next_qr_pos)
		position = next_qr_pos.pose.position
		orientation = next_qr_pos.pose.orientation
		pos_x, pos_y, pos_z = position.x, position.y, position.z
		orien_x, orien_y, orien_z = orientation.x, orientation.y, orientation.z

		if pos_x != 0. and pos_y != 0. and pos_z != 0. and qr_message.data != "":	
			msg_list = [i.rsplit("=",1)[1] for i in qr_message.data.split("\r\n")]
			curr_qr_pos = [float(msg_list[0]), float(msg_list[1])]
			next_qr_pos = [float(msg_list[2]), float(msg_list[3])]
			num_qr = float(msg_list[4])
			msg_qr = msg_list[5]
			return [[curr_qr_pos, next_qr_pos, num_qr, msg_qr], [position, orientation]]
	else:
		return None
	