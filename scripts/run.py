#!/usr/bin/env python
# BEGIN ALL
import rospy
import time
import numpy as np
import pdb
import math
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


def shutdown_hook():
    print("Shutdown rospy...")


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
                rospy.sleep(5.)
                # avoiding error TypeError: time must have a to_sec method, e.g. rospy.Time or rospy.Duration
                now = transform.get_qr_code()
                # now_sec = now_out.to_sec()
                # now = rospy.Time(now_sec)
                rospy.sleep(5.)
                # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
                current_qr_numb = scan_output[0][2]
                current_qr_locat = scan_output[0][0]
                next_qr_locat = scan_output[0][1]
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
        else:
            robot.stop()
            print('Two QR codes found.')
            bool_random_search = False
            break


    #################################################################
    ## helper functions
    #################################################################
    def run_to_goal(goal_, id_):
        print("Goal with offset:{}".format(goal_))
        qr_id = int(id_.split("/")[0])
        robot.move_to_goal(goal_, "map", id_)
        rospy.sleep(5.)
        goal_set = True
        while goal_set is True:
            if status_goal == 4:  # goal status 4 "Goal not reachable"
                print("Goal not reachable...")
                robot.cancel_goal(id_)
                rospy.sleep(5.)
                print("...cancel goal with id: {}".format(id_))
                return False, None
            elif status_goal == 3 and id_ == status_id:  # if goal reached
                print("Goal reached: QR Code number {}".format(qr_id))
                # scanning of new qr code
                scan_output = scan.scan(search_id=qr_id, specific_search=True)
                if scan_output is not None and int(scan_output[0][2]) == qr_id:
                    print("Found qr code after reaching goal")
                    return True, scan_output
                else:
                    print("Reached goal but not able to scan")
                    time_threshold = 15.  # time robot needs to spin once with speed of 0.05
                    print("start spinning once")
                    robot.spin_360(spinning_speed=0.5 * math.pi / time_threshold)  # 1. turn slowly by 360 degrees
                    start_time = time.time()
                    end_time = 0
                    while (scan_output is None) and (
                            end_time - start_time < time_threshold * 4):  # TODO how to get time for one spin?
                        scan_output = scan.scan(search_id=qr_id, specific_search=True)
                        if scan_output is not None and int(scan_output[0][2]) == int(id_.split("/")[0]):
                            break
                        rospy.sleep(1.)
                        end_time = time.time()
                    print("Stop robot after spinning")
                    robot.stop()
                    rospy.sleep(2.)
                    if scan_output is not None and int(scan_output[0][2]) == int(
                            id_.split("/")[0]):  # during spinning qr code found
                        print("Found qr code during spinning")
                        robot.stop()
                        rospy.sleep(3.)
                        return True, scan_output
                    elif (end_time - start_time) >= time_threshold * 4:  # after spinning no qr code found
                        print("After spinning no qr code found")
                        return False, None

    def get_infos(s_output):
        now_ = transform.get_qr_code()
        #now_sec = now_out.to_sec()
        #now_ = rospy.Time(now_sec)
        rospy.sleep(5.)
        # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
        current_qr_numb_ = s_output[0][2]
        current_qr_locat_ = s_output[0][0]
        next_qr_locat_ = s_output[0][1]
        listener.waitForTransform('/map', 'qr_frame', now_, rospy.Duration(4.0))
        try:
            listener.waitForTransform('/map', 'qr_frame', now_, rospy.Duration(4.0))
            (trans1_, rot1_) = listener.lookupTransform('/map', '/qr_frame', now_)
            delta_qr_ = np.array(next_qr_locat_) - np.array(current_qr_locat_)
            # rot_euler = tf.transformations.euler_from_quaternion(rot1)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
        if current_qr_numb_ == 5:
            next_qr_numb_ = 1
        else:
            next_qr_numb_ = current_qr_numb_ + 1
        dict_global_qr_codes[current_qr_numb_] = [[trans1_, delta_qr_], next_qr_numb_]
        rospy.sleep(3.)

    def goal_runner(goals_list_, goal_qr_id_, offset=1.2):
        success_ = False
        try_counter = 0
        while success_ is False and try_counter < 4:
            for num_tries_, curr_goal_ in enumerate(goals_list_):
                if success_ is False:
                    print("Set goal with offset option number {}".format(num_tries_ + 1))
                    success_, scan_output_ = run_to_goal(curr_goal_, str(goal_qr_id_) + "/" + str(try_counter) + str(num_tries_ + 1))
                elif success_:
                    print("##############")
                    print("SUCCESS")
                    print("##############")
                    get_infos(scan_output_)
                    return True
            print("COULD NOT REACH QR CODE NUMBER {}".format(goal_qr_id_))
            try_counter += 1
            offset -= .1
        return False

    #################################################################
    ## focused search
    #################################################################
    transform.get_hidden_frame(now)
    goal_set = False
    # Focused search
    qr_goal_index = 0
    print("\n####################")
    print("start focused search")
    print("####################")
    while bool_random_search is False and goal_set is False:
        # check if current qr goal already read
        qr_goal_index = 0
        read_qr_codes = dict_global_qr_codes.keys()
        while dict_global_qr_codes[read_qr_codes[qr_goal_index]][1] in read_qr_codes:
            qr_goal_index += 1
            if qr_goal_index == 5:
                qr_goal_index = 0
        goal_qr_id = int(dict_global_qr_codes[read_qr_codes[qr_goal_index]][1])
        print("New goal: QR Code: {}".format(goal_qr_id))
        trans, delta = dict_global_qr_codes[read_qr_codes[qr_goal_index]][0]
        goal = np.array(trans) + np.array([-delta[1], delta[0], 0])
        goal[2] = 0  # set z value to ground floor
        offset = 1.2
        x_goal = goal[0]
        y_goal = goal[1]
        if x_goal < 0 and y_goal < 0:  # third quadrant of map frame
            print("Goal in third quadrant")
            x_goal1 = x_goal + offset
            x_goal2 = x_goal - offset
            y_goal1 = y_goal + offset
            y_goal2 = y_goal - offset
            goals_list = [[goal[0], y_goal1], [goal[0], y_goal2], [x_goal1, goal[1]], [x_goal2, goal[1]]]
        elif x_goal < 0 and y_goal > 0:  # fourth quadrant of map frame
            print("Goal in fourth quadrant")
            x_goal1 = x_goal + offset
            x_goal2 = x_goal - offset
            y_goal1 = y_goal - offset
            y_goal2 = y_goal + offset
            goals_list = [[goal[0], y_goal1], [goal[0], y_goal2], [x_goal1, goal[1]], [x_goal2, goal[1]]]
        elif x_goal > 0 and y_goal > 0:
            print("Goal in first quadrant")
            x_goal1 = x_goal + offset
            x_goal2 = x_goal - offset
            y_goal1 = y_goal - offset
            y_goal2 = y_goal + offset
            goals_list = [[goal[0], y_goal1], [goal[0], y_goal2], [x_goal1, goal[1]], [x_goal2, goal[1]]]
        elif x_goal > 0 and y_goal < 0:
            print("Goal in second quadrant")
            x_goal1 = x_goal + offset
            x_goal2 = x_goal - offset
            y_goal1 = y_goal - offset
            y_goal2 = y_goal + offset
            goals_list = [[goal[0], y_goal1], [goal[0], y_goal2], [x_goal1, goal[1]], [x_goal2, goal[1]]]
        goal_feedback = goal_runner(goals_list_=goals_list, goal_qr_id_=goal_qr_id)
        if goal_feedback is False:
            print("No possible solution found")
            break
        elif len(dict_global_qr_codes.keys()) == 5:
            print("Finished task successful")
            break
    rospy.on_shutdown(shutdown_hook)
    break
