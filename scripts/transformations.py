#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage

import tf
from tf.transformations import euler_from_quaternion

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

class Transformations:

	def __init__(self):
		self.current_tf = None
		self.current_tf_static = None
		self.tf_sub = rospy.Subscriber('/tf', TFMessage, self._TFCallback)
		self.tf_static_sub = rospy.Subscriber('/tf_static', TFMessage, self._TFStaticCallback)

	def get_tf_transformation(self, frame_id, child_frame_id):
		while True:
			if self.current_tf is not None and \
				self.current_tf.transforms[0].header.frame_id == frame_id and \
				self.current_tf.transforms[0].child_frame_id == child_frame_id:
				return self._get_tf_matrix(self.current_tf)
				
	def get_tf_static_transformation(self, frame_id, child_frame_id):
		while True:
			self.tf_sub
			if self.current_tf_static is not None and \
				self.current_tf_static.transforms[0].header.frame_id == frame_id and \
				self.current_tf_static.transforms[0].child_frame_id == child_frame_id:
				return self._get_tf_matrix(self.current_tf_static)

	def _get_tf_matrix(self, tf_msg):
		translation = tf_msg.transforms[0].transform.translation
		translation = [translation.x, translation.y, translation.z]
		rotation_quat = tf_msg.transforms[0].transform.rotation
		quaternion = [rotation_quat.x, rotation_quat.y, rotation_quat.z, rotation_quat.w]
		euler_angel = euler_from_quaternion(quaternion)
		rotation_matrix = tf.transformations.euler_matrix(euler_angel[0], euler_angel[1], euler_angel[2])
		tf_matrix = rotation_matrix
		tf_matrix[:3,-1] = translation	
		return tf_matrix	

	def _TFCallback(self, msg):
		self.current_tf = msg

	def _TFStaticCallback(self, msg):
		self.current_tf_static = msg