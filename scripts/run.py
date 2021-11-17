#!/usr/bin/env python
# BEGIN ALL
import rospy
import numpy as np
import pdb
import tf
from navigation import Robot
from scan import Scan
from transformations import Transformation
from actionlib_msgs.msg import GoalStatusArray

rospy.init_node('robot')

robot = Robot()
scan = Scan()
trans = Transformation()

bool_random_search = True
scan_output = None
setting_target_goal = False
dict_global_qr_codes = {}
dict_hidden_qr_codes = {}

listener = tf.TransformListener()


def callback_status(msg):
    global status_goal
    global status_id
    if len(msg.status_list) > 0:
        status_goal = msg.status_list[0].status
        status_id = msg.status_list[0].goal_id.id


rospy.Subscriber("move_base/status", GoalStatusArray, callback_status)
i = 0
while not rospy.is_shutdown():

    ## Start-phase: Robot should find two QR codes with random search

    while bool_random_search is True:
        if len(dict_global_qr_codes) < 2:
            robot.random_search()
            scan_output = scan.scan()

            if scan_output is not None:
                robot.stop()
                rospy.sleep(5.)
                now = trans.get_qr_code()
                rospy.sleep(5.)
                # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
                current_qr_numb = scan_output[0][2]
                current_qr_locat = scan_output[0][0]
                next_qr_locat = scan_output[0][1]
                listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
                try:
                    #now = rospy.Time.now()
                    listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
                    (trans1, rot1) = listener.lookupTransform('/map', '/qr_frame', now)
                    print(trans1)
                    delta_qr = np.array(next_qr_locat) - np.array(current_qr_locat)
                    rot_euler = tf.transformations.euler_from_quaternion(rot1)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)
                dict_global_qr_codes[current_qr_numb] = [trans1, delta_qr]
                rospy.sleep(3.)
        else:
            robot.stop()
            print('Two QR codes found.')
            bool_random_search = False
            break
    trans.get_hidden_frame(now)
    goal_set = False
    # Focused search
    list_key = dict_global_qr_codes.keys()
    i = 0
    while bool_random_search is False and goal_set is False:
        trans, delta = dict_global_qr_codes[list_key[i]]

        goal = np.array(trans) + np.array([-delta[1], delta[0], 0])
        goal[2] = 0
        # offset
        offset = 0.7
        if goal[0] < 0 and goal[1] < 0:
            goal[0] += offset
            goal[1] += offset
        elif goal[0] < 0 and goal[1] > 0:
            goal[0] += offset
            goal[1] -= offset
        goal_set = True

    robot.move_to_goal(goal, "map", str(list_key[i]))
    rospy.sleep(1.)
    print("Goal:{}".format(goal))
    while bool_random_search is False and goal_set is True:
        if status_goal == 3 and str(list_key[i]) == status_id:
            goal_set = False
            i += 1

    # if current_qr_numb+1 not in dict_global_qr_codes.keys():
    #	goal = np.array(next_qr_locat)-np.array([0.5,0.5])
    #	robot.move_to_goal(goal, "qr_coordinate_frame")

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


# while scan_output is not None:
#	rospy.sleep(2) # Sleeps for 1 sec
#	print('Sleeping over')
#	#print('Setting goal to: {}'.format(scan_output[1]))
#	print(scan_output[0][1])
#	robot.move_to_goal(scan_output[0][1], "qr_coordinate_frame")
#	#rans.get_qr_code()
#	#success = trans.get_hidden_frame()
