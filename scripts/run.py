#!/usr/bin/env python
# BEGIN ALL
import rospy
import time
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

    #################################################################
    ## Start-phase: Robot should find two QR codes with random search
    #################################################################
    while bool_random_search is True:
        if len(dict_global_qr_codes) < 2:
            robot.random_search()
            scan_output = scan.scan()

            if scan_output is not None:
                robot.stop()
                # TODO: Let robot move to qr in such a way that orientation is 90degree and move to it with
                #  offset to it on a straight line and then get transform (so it stabilizes) --> only necessary if with
                #  actual random search scanning is still inaccurate
                rospy.sleep(5.)
                now = rospy.Duration()  # avoiding error TypeError: time must have a to_sec method, e.g. rospy.Time or rospy.Duration
                now = transform.get_qr_code()
                rospy.sleep(5.)
                # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
                current_qr_numb = scan_output[0][2]
                current_qr_locat = scan_output[0][0]
                next_qr_locat = scan_output[0][1]
                listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
                try:
                    listener.waitForTransform('/map', 'qr_frame', now, rospy.Duration(4.0))
                    (trans1, rot1) = listener.lookupTransform('/map', '/qr_frame', now)
                    print(trans1)
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
        else:
            robot.stop()
            print('Two QR codes found.')
            bool_random_search = False
            break

    #################################################################
    ##
    #################################################################
    def run_to_goal(goal_, id_):
        print("Goal:{}".format(goal_))
        robot.move_to_goal(goal_, "map", id_)
        rospy.sleep(1.)
        goal_set = True
        while goal_set is True:
            # print("Status:", status_goal)
            # print("Status id", id_, " Status id msg:", status_id)
            if status_goal == 4:  # goal status 4 "Goal not reachable"
                # TODO stop goal movement
                print("goal not reachable")
                return False
            elif status_goal == 3 and id_ == status_id:  # if goal reached
                print("Goal reached: QR Code number {}".format(read_qr_codes[qr_goal_index]))
                # scanning of new qr code
                scan_output = scan.scan()
                if scan_output is not None and int(scan_output[0][2]) == int(id_.split("/")[0]):
                    # TODO get information
                    print("found qr code after reaching goal")
                    return True
                else:
                    print("reached goal but not able to scan")
                    # TODO Adjust robot position if scanning not possible
                    time_threshold = 20.  # time robot needs to spin once with speed of 0.05
                    print("start spinning once")
                    robot.spin_360(spinning_speed=1/time_threshold)  # 1. turn slowly by 360 degrees
                    start_time = time.time()
                    end_time = 0
                    while scan_output is None or (end_time - start_time) < time_threshold:
                        scan_output = scan.scan(search_id=)
                        if scan_output is not None and int(scan_output[0][2]) == int(id_.split("/")[0]):
                            robot.stop()
                        rospy.sleep(1.)
                        end_time = time.time()
                    if scan_output is not None and int(scan_output[0][2]) == int(id_.split("/")[0])+1:  # during spinning qr code found
                        # TODO get information
                        print("found qr code during spinning")
                        return True
                    elif (end_time - start_time) >= time_threshold:  # after spinning no qr code found
                        print(int(scan_output[0][2]))
                        print(int(id_.split("/")[0]))
                        print("after spinning no qr code found")
                        return False


    transform.get_hidden_frame(now)
    goal_set = False
    # Focused search
    read_qr_codes = dict_global_qr_codes.keys()
    qr_goal_index = 0
    print("start focused search")
    while bool_random_search is False and goal_set is False:
        # check if current qr goal already read
        qr_goal_index = 0
        while dict_global_qr_codes[read_qr_codes[qr_goal_index]][1] in read_qr_codes:
            qr_goal_index += 1
        print("New goal: QR Code: {}".format(read_qr_codes[qr_goal_index]+1))
        trans, delta = dict_global_qr_codes[read_qr_codes[qr_goal_index]][0]
        goal = np.array(trans) + np.array([-delta[1], delta[0], 0])
        goal[2] = 0  # set z value to ground floor
        offset = 1.5
        x_goal = goal[0]
        y_goal = goal[1]
        if x_goal < 0 and y_goal < 0:  # third quadrant of map frame
            x_goal += offset
            y_goal += offset
            goals_list = [[goal[0], y_goal], [x_goal, goal[1]], [-x_goal, goal[1]]]
            success = False
            for num_tries, curr_goal in enumerate(goals_list):
                if success is False:
                    success = run_to_goal(curr_goal, str(int(read_qr_codes[qr_goal_index])) + "/" + str(num_tries+1))
                else:
                    print("##############")
                    print("SUCCESS")
                    print("##############")
                    break
        #goal_set = True
    rospy.spin()
    """
    robot.move_to_goal(goal, "map", str(read_qr_codes[qr_goal_index]))
    rospy.sleep(1.)
    print("Goal:{}".format(goal))
    while bool_random_search is False and goal_set is True:
        # TODO: goal status 4 "Goal not reachable"

        if status_goal == 3 and str(read_qr_codes[i]) == status_id:  # if goal reached
            print("Goal reached: QR Code number {}".format(read_qr_codes[qr_goal_index]))
            # scanning of new qr code
            scan_output = scan.scan()
            if scan_output is not None:
                pass
                # TODO get information
            else:
                pass
                # TODO Adjust robot position if scanning not possible
                # 1. turn slowly by 360 degrees
                # 2. further adjustments
            goal_set = False
    """



