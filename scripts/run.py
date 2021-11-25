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
transform = Transformation()


search_qr_codes = True
bool_random_search = False
scan_output = None
setting_target_goal = False
dict_global_qr_codes = {}
dict_hidden_qr_codes = {}
spinning_time = rospy.Duration(secs=10)
listener = tf.TransformListener()

status_id = ''

def callback_status(msg):
    global status_goal
    global status_id
    if len(msg.status_list)>0:
        status_goal = msg.status_list[0].status
        status_id = msg.status_list[0].goal_id.id


rospy.Subscriber("move_base/status", GoalStatusArray, callback_status)

i = 0



while not rospy.is_shutdown():
    #print('Starting random search')
    while search_qr_codes is True:
        if len(dict_global_qr_codes) < 2:
            robot.focused_search()
            #rospy.sleep(2.)
            print(status_goal)
            if status_goal == 3:

                print('Status 3')
                start_timer = rospy.get_rostime()
                finish_timer = rospy.get_rostime()
                print('####################')
                print('Exploring Environment by turning')
                print('####################')
        
                while (finish_timer-start_timer)<spinning_time:
                    
                    robot.turn()
                    scan_output = scan.scan()
                    finish_timer = rospy.get_rostime()
                    if scan_output is not None:
                            print('Scanning QR Code')
                            robot.stop()
                            rospy.sleep(2.)
                            # avoiding error TypeError: time must have a to_sec method, e.g. rospy.Time or rospy.Duration
                            now = transform.get_qr_code()
                            # now_sec = now_out.to_sec()
                            # now = rospy.Time(now_sec)
                            rospy.sleep(2.)
                            current_qr_numb = scan_output[0][2]
                            current_qr_locat = scan_output[0][0]
                            next_qr_locat = scan_output[0][1]
                            try:
                                listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
                                try:
                                    listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
                                    (trans1, rot1) = listener.lookupTransform('/map', '/qr_frame', now)
                                    delta_qr = np.array(next_qr_locat) - np.array(current_qr_locat)
                                    rot_euler = tf.transformations.euler_from_quaternion(rot1)
                                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                                    print(e)
                                if current_qr_numb == 5:
                                    next_qr_numb = 1
                                else:
                                    next_qr_numb = current_qr_numb + 1
                                dict_global_qr_codes[current_qr_numb] = [[trans1, delta_qr], next_qr_numb]
                                rospy.sleep(3.)
                            except TypeError as e:
                                print("Found QR Code but transform listener failed...")
                                # delete latest qr scan after failed transform listening
                                del scan.qr_dict[scan.num_qr]
                                print(scan.qr_dict)
                                print("...continue random search!")
        else:
            robot.stop()
            print('Two QR codes found.')
            search_qr_codes = False
            robot.canceling_goal()
            break

#
            #elif status_goal ==2:
            #    scan_output = scan.scan()
            #    if scan_output is not None:
            #       robot.stop()
            #       print('QR detection in progress')
            #       rospy.sleep(10.)


            #scan.validate_scan()
            #scan_output = scan.scan()
            #if scan.validate_scan() is True:
            #    robot.canceling_goal()
            #    print('Goal canceled')




    #################################################################
    ## Start-phase: Robot should find two QR codes with random search
    #################################################################
            
   #         
   #         scan_output = scan.scan()
   #         if scan_output is not None:
   #             #robot.stop()
   #             print('QR detection in progress')
   #             rospy.sleep(10.)
   #             
   #             now = transform.get_qr_code()
   #             rospy.sleep(10.)
   #             # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
   #             current_qr_numb = scan_output[0][2]
   #             current_qr_locat = scan_output[0][0]
   #             next_qr_locat = scan_output[0][1]
   #             listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
   #             try:
   #                 #now = rospy.Time.now()
   #                 listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
   #                 (trans1, rot1) = listener.lookupTransform('/map', '/qr_frame', now)
   #                 print(trans1)
   #                 delta_qr = np.array(next_qr_locat) - np.array(current_qr_locat)
   #                 rot_euler = tf.transformations.euler_from_quaternion(rot1)
   #             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
   #                 print(e)
   #             if current_qr_numb == 5:
   #                 next_qr_numb = 1
   #             else:
   #                 next_qr_numb = current_qr_numb + 1
   #             dict_global_qr_codes[current_qr_numb] = [[trans1, delta_qr], next_qr_numb]
   #             rospy.sleep(3.)
   #     else:
   #         robot.stop()
   #         print('Two QR codes found.')
   #         search_qr_codes = False
   #         break
    #################################################################
    ##
#    #################################################################
#    transform.get_hidden_frame(now)
#    goal_set = False
#    
#
#
#    # Focused search
#    read_qr_codes = dict_global_qr_codes.keys()
#    qr_goal_index = 0
#    while bool_random_search is False and goal_set is False:
#        # check if current qr goal already read
#        while dict_global_qr_codes[read_qr_codes[qr_goal_index]][1] in read_qr_codes:
#            qr_goal_index += 1
#        trans, delta = dict_global_qr_codes[read_qr_codes[qr_goal_index]][0]
#        goal = np.array(trans) + np.array([-delta[1], delta[0], 0])
#        goal[2] = 0
#        # offset
#        # TODO: seperate offset in 4 scenarios (goal +/- x-offset; goal+/- y-offset)
#        offset = 0.7
#        if goal[0] < 0 and goal[1] < 0:
#            goal[0] += offset
#            goal[1] += offset
#        elif goal[0] < 0 and goal[1] > 0:
#            goal[0] += offset
#            goal[1] -= offset
#        goal_set = True
#    robot.move_to_goal(goal, "map", str(read_qr_codes[qr_goal_index]))
#    rospy.sleep(1.)
#    print("Goal:{}".format(goal))
#    while bool_random_search is False and goal_set is True:
#        # TODO: goal status 4 "Goal not reachable"
#        if status_goal == 3 and str(read_qr_codes[i]) == status_id: # if goal reached
#            print("Goal reached: QR Code number {}".format(read_qr_codes[qr_goal_index]))
#            # scanning of new qr code
#            scan_output = scan.scan()
#            if scan_output is not None:
#                pass
#                # TODO get information
#            else:
#                pass
#                # TODO Adjust robot position if scanning not possible
#                # 1. turn slowly by 360 degrees
#                # 2. further adjustments
#            goal_set = False
#   rospy.spin()