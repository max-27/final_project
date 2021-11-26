#!/usr/bin/env python
# BEGIN ALL
import rospy
import time
import numpy as np
import pdb
import math
import tf
from robot import Robot
from scan import Scan
from transformations import Transformation
from actionlib_msgs.msg import GoalStatusArray
import settings
from tf.transformations import euler_from_quaternion


def callback_status(msg):
    global status_goal
    global status_id
    if len(msg.status_list) > 0:
        status_goal = msg.status_list[0].status
        status_id = msg.status_list[0].goal_id.id


def shutdown_hook():
    print("Shutdown rospy...")


rospy.init_node('robot')
robot = Robot()
scan = Scan()
transform = Transformation()
random_search = True
scan_output = None
setting_target_goal = False
dict_global_qr_codes = {}
dict_hidden_qr_codes = {}
listener = tf.TransformListener()
rospy.Subscriber("move_base/status", GoalStatusArray, callback_status)
while not rospy.is_shutdown():
    #################################################################
    # Start-phase: Robot should find two QR codes with random search
    #################################################################
    while random_search is True:
        if len(dict_global_qr_codes) < 2:
            robot.random_search()
            scan_output = scan.scan()
            if scan_output is not None:
                robot.stop()
                rospy.sleep(3.)
                if transform.code is not "":  # check if after stopping QR is still in reach
                    now = transform.get_qr_code()
                    rospy.sleep(5.)
                    current_qr_numb = scan_output[0][2]
                    current_qr_locat = scan_output[0][0]
                    next_qr_locat = scan_output[0][1]
                    message = scan_output[0][-1]
                    try:
                        if current_qr_numb == 5:
                            next_qr_numb = 1
                        else:
                            next_qr_numb = current_qr_numb + 1
                        next_qr_locat.append(0)
                        dict_global_qr_codes[current_qr_numb] = [[message, next_qr_locat], next_qr_numb]
                        rospy.sleep(3.)
                    except TypeError as e:
                        # avoiding error TypeError: time must have a to_sec method, e.g. rospy.Time or rospy.Duration
                        print("Found QR Code but transform listener failed...")
                        # delete latest qr scan after failed transform listening
                        del scan.qr_dict[scan.num_qr]
                        print(scan.qr_dict)
                        print("...continue random search!")
                else:
                    del scan.qr_dict[scan.num_qr]
                    print(scan.qr_dict)
                    print("QR is out  of reach after stopping: Continue search")
        else:
            robot.stop()
            print('Two QR codes found.')
            random_search = False
            break

    #################################################################
    # helper functions
    #################################################################
    def run_to_goal(goal_, id_):
        print("Goal with offset:{}".format(goal_))
        qr_id = int(id_.split("/")[0])
        robot.move_to_goal(goal_, "map", id_)
        rospy.sleep(10.)
        goal_set_ = True
        while goal_set_:
            if status_goal == 4:  # goal status 4 "Goal not reachable"
                print("Goal not reachable...")
                robot.cancel_goal(id_)
                robot.stop()
                rospy.sleep(5.)
                print("...cancel goal with id: {}".format(id_))
                return False, None
            elif status_goal == 3 and id_ == status_id:  # if goal reached
                rospy.sleep(4.)
                print(status_goal)
                print("Goal reached: QR Code number {}".format(qr_id))
                # scanning of new qr code
                scan_output_ = scan.scan(search_id=qr_id, specific_search=True)
                if scan_output_ is not None and int(scan_output_[0][2]) == qr_id:
                    print("Found qr code after reaching goal")
                    return True, scan_output_
                else:
                    print("Reached goal but not able to scan")
                    print("Start spinning once")
                    robot.spin_360(spinning_speed=settings.SPIN_SPEED)
                    start_time = time.time()
                    end_time = 0
                    while (scan_output_ is None) and (
                            end_time - start_time < settings.SPIN_TIME * 5):  # TODO how to get time for one spin?
                        scan_output_ = scan.scan(search_id=qr_id, specific_search=True)
                        if scan_output_ is not None and int(scan_output_[0][2]) == qr_id:
                            break
                        rospy.sleep(1.)
                        end_time = time.time()
                    print("Stop robot after spinning")
                    robot.stop()
                    rospy.sleep(3.)
                    if scan_output_ is not None and int(scan_output_[0][2]) == int(id_.split("/")[0]):
                        print("Found qr code during spinning")
                        robot.stop()
                        rospy.sleep(3.)
                        return True, scan_output_
                    elif (end_time - start_time) >= settings.SPIN_TIME * 5:  # after spinning no qr code found
                        print("After spinning no qr code found")
                        return False, None


    def get_infos(s_output):
        transform.get_qr_code()
        rospy.sleep(5.)
        # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
        current_qr_numb_ = s_output[0][2]
        next_qr_locat_ = s_output[0][1]
        message = scan_output[0][-1]
        if current_qr_numb_ == 5:
            next_qr_numb_ = 1
        else:
            next_qr_numb_ = current_qr_numb_ + 1
        next_qr_locat_.append(0)
        dict_global_qr_codes[current_qr_numb_] = [[message, next_qr_locat_], next_qr_numb_]
        rospy.sleep(3.)


    def goal_runner(goal_, goal_qr_id_):
        x_goal = goal_[0]
        y_goal = goal_[1]
        success_ = False
        try_counter = 0
        while success_ is False and try_counter < len(settings.OFFSETS):
            print("\nRun goal approach with offset of: {}".format(settings.OFFSETS[try_counter]))
            x = [x_goal + settings.OFFSETS[try_counter], x_goal - settings.OFFSETS[try_counter]]
            y = [y_goal + settings.OFFSETS[try_counter], y_goal - settings.OFFSETS[try_counter]]
            goals_list_ = [[x_goal, y[0]], [x_goal, y[1]], [x[0], y_goal], [x[1], y_goal],
                           [x[0], y[0]], [x[0], y[1]], [x[1], y[0]], [x[1], y[1]]]
            for num_tries_, curr_goal_ in enumerate(goals_list_):
                if success_ is False:
                    print("Set goal with offset option number {}".format(num_tries_ + 1))
                    success_, scan_output_ = run_to_goal(curr_goal_, str(goal_qr_id_) + "/" + str(try_counter) + str(
                        num_tries_ + 1))
                if success_:
                    get_infos(scan_output_)
                    curr_msg = get_secret_msg(dict_global_qr_codes)
                    print("\n##############################################################")
                    print("# Success: QR {} scanned".format(goal_qr_id_))
                    print("# Current secret message: {}".format(curr_msg))
                    print("##############################################################")
                    return True
                rospy.sleep(3.)
            print("Could not reach qr code {} with current offset settings".format(goal_qr_id_))
            try_counter += 1
        print("NOT ABLE TO REACH QR CODE {}!".format(goal_qr_id_))
        return False


    def get_secret_msg(dict_):
        return "".join([m[0][0] for m in dict_.values()])


    #################################################################
    # focused search
    #################################################################
    now_map_hidden = transform.get_hidden_frame(now)
    goal_set = False
    # Focused search
    qr_goal_index = 0
    curr_msg = get_secret_msg(dict_global_qr_codes)
    print("\n##############################################################")
    print("# Start focused search")
    print("# Current secret message: {}".format(curr_msg))
    print("##############################################################")
    listener.waitForTransform('/map', 'qr_coordinate_frame', rospy.Time.now(), rospy.Duration(4.0))
    try:
        listener.waitForTransform('/map', 'qr_coordinate_frame', rospy.Time.now(), rospy.Duration(4.0))
        (trans_map_hidden, rot_map_hidden) = listener.lookupTransform('/map', '/qr_coordinate_frame', rospy.Time.now())
        euler_angle = euler_from_quaternion(rot_map_hidden)
        rotation_matrix = tf.transformations.euler_matrix(euler_angle[0], euler_angle[1], euler_angle[2])[:3, :3]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)

    while random_search is False and goal_set is False:
        # check if current qr goal already read
        qr_goal_index = 0
        read_qr_codes = dict_global_qr_codes.keys()
        while dict_global_qr_codes[read_qr_codes[qr_goal_index]][1] in read_qr_codes:
            qr_goal_index += 1
            if qr_goal_index == 5:
                qr_goal_index = 0
        goal_qr_id = int(dict_global_qr_codes[read_qr_codes[qr_goal_index]][1])
        print("New goal: QR Code: {}".format(goal_qr_id))
        trans, goal_hidden = dict_global_qr_codes[read_qr_codes[qr_goal_index]][0]
        goal_map = np.matmul(rotation_matrix,
                             goal_hidden) + trans_map_hidden  # rotate and translate goal into map frame
        goal_map[2] = 0  # set z value to ground floor
        print("Goal without offset: {}".format(goal_map))
        goal_feedback = goal_runner(goal_=goal_map, goal_qr_id_=goal_qr_id)
        if goal_feedback is False:
            print("No possible solution found")
            break
        elif len(dict_global_qr_codes.keys()) == 5:
            final_msg = get_secret_msg(dict_global_qr_codes)
            print("Final encoded message: {}".format(final_msg))
            print("Finished task successful")
            break
    rospy.on_shutdown(shutdown_hook)
    break
