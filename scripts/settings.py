# definition of fixed parameters
import math
import rospy

OFFSETS = [1., 0.6, 1.4, 0.8, 1.6]
SPIN_TIME = 15
SPIN_TIME_SELECT_SEARCH = rospy.Duration(secs=45)
SPIN_SPEED = 0.5 * math.pi / SPIN_TIME

