#!/usr/bin/env python
# BEGIN ALL
import rospy

from navigation import move


while not rospy.is_shutdown():
	move()