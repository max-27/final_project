#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import numpy as np  # For random numbers
import tf
import time
import math
import geometry_msgs.msg


class Transformation:

	def __init__(self):
		self.pose = None
		self.code = ''
		self.d = []
		self.sec = []
		self.number_qr = None
		rospy.Subscriber('visp_auto_tracker/code_message', String, self._code_cal)
		rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, self._obj_cal)
		self.br = tf.TransformBroadcaster()
		self.listener = tf.TransformListener()  # to get translation & rotations between the frames
		self.pr = tf.TransformBroadcaster()  # to publish new frames
		self.trs = {}  # dictionary for storing all info from the qr

# Create callback. This is what happens when a new message is received
	def _obj_cal(self, msg):
		self.pose = msg.pose

	def _code_cal(self, msg):
	# import pdb; pdb.set_trace()
		self.code = msg.data  # to get output from the qr code
		if self.code is not '':  # split the output & use it afterwards
			fir = self.code.split("\r\n")
			self.sec = []
			for i, e in enumerate(fir):
				self.sec.append(e.split("="))
			self.number_qr = self.sec[-2][1]

	def get_qr_code(self):
		if self.code is not '':
			now = rospy.Time.now()
			self.br.sendTransform([self.pose.position.x, self.pose.position.y, self.pose.position.z],
									[self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z,
									self.pose.orientation.w],
									now,
									"qr_frame",
									"camera_optical_link")
			#rospy.sleep(3.)
			self.listener.waitForTransform("/map", "/qr_frame", now, rospy.Duration(4.0))
			try:
				# http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29
				self.listener.waitForTransform("/map", "/qr_frame", now, rospy.Duration(4.0))
				(trans1, rot1) = self.listener.lookupTransform('/map', '/qr_frame', now)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				print(e)

			# Use qr_code to map frame translations
			self.pr.sendTransform(trans1,
									[0.0, 0.0, 0.0, 1.0],
									now,
									"tr_frame",
									"map")
			qr_name = 'qr_%d' % float(self.number_qr)

			if qr_name not in self.trs:
				self.trs[qr_name] = [trans1, float(self.sec[0][1]), float(self.sec[1][1]), float(self.sec[2][1]),
									 float(self.sec[3][1]),
									 float(self.sec[4][1]), self.sec[5][1]]
				self.d.append(qr_name)
			return now

	def get_hidden_frame(self, timestamp):

		if len(self.trs) == 2:  # if you detect two qr codes
			# name = trs.keys() #list of keys for the dictionary
			qr1_coords_world = self.trs[self.d[0]][0]
			qr2_coords_world = self.trs[self.d[1]][0]
			qr1_coords_qr = self.trs[self.d[0]][1:3]
			qr2_coords_qr = self.trs[self.d[1]][1:3]

			world_x = qr2_coords_world[0] - qr1_coords_world[0]  # create vector
			world_y = qr2_coords_world[1] - qr1_coords_world[1]
			distance_w = math.sqrt(world_x ** 2 + world_y ** 2)  # Modulus
			theta_w = math.acos(world_y / distance_w)  # Angle of rotation
		if world_x < 0.0: theta_w = 2 * math.pi - theta_w

		qr_x = qr2_coords_qr[0] - qr1_coords_qr[0]
		qr_y = qr2_coords_qr[1] - qr1_coords_qr[1]
		distance_qr = math.sqrt(qr_x ** 2 + qr_y ** 2)  # Modulus
		theta_qr = math.acos(qr_y / distance_qr)  # Angle of rotation
		if qr_x < 0.0: theta_qr = 2 * math.pi - theta_qr

		final_theta = theta_qr - theta_w  # rotation between qr codes frame and map frame
		print("Angle between hidden and map frame:", final_theta)

		now = timestamp
		# to get zero rotation wrt qr codes frame
		self.pr.sendTransform([0.0, 0.0, 0.0], tf.transformations.quaternion_from_euler(0.0, 0.0, final_theta),
							now,
							"rr_frame",
							"tr_frame")
		#now = rospy.Time.now()
		# translate to qr codes frame, Bingo
		self.pr.sendTransform([-qr2_coords_qr[0], -qr2_coords_qr[1], 0.0],
							tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
							now,
							"qr_pre_frame",
							"rr_frame")

		# now you need to save wrt something static, because it vanishes after you move
		self.listener.waitForTransform("/map", "/qr_pre_frame", timestamp, rospy.Duration(4.))
		success = False
		try:
			#now = rospy.Time.now()
			self.listener.waitForTransform('/map', 'qr_pre_frame', now, rospy.Duration(4.0))
			(trans2, rot2) = self.listener.lookupTransform('/map', '/qr_pre_frame', now)
			success = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			print(e)

		final_now = rospy.Time.now()
		self.pr.sendTransform(trans2, rot2,
								final_now,
								"qr_coordinate_frame",
								"map")
		return final_now
