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


class Goal:

    def __init__(self):
        pass


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
                        end_time - start_time < time_threshold * 5):  # TODO how to get time for one spin?
                    scan_output = scan.scan(search_id=qr_id, specific_search=True)
                    if scan_output is not None and int(scan_output[0][2]) == qr_id:
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
    rospy.sleep(5.)
    # return of scan.scan [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
    current_qr_numb_ = s_output[0][2]
    next_qr_locat_ = s_output[0][1]
    if current_qr_numb_ == 5:
        next_qr_numb_ = 1
    else:
        next_qr_numb_ = current_qr_numb_ + 1
    next_qr_locat_.append(0)
    trans1_ = None
    dict_global_qr_codes[current_qr_numb_] = [[trans1_, next_qr_locat_], next_qr_numb_]
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
                success_, scan_output_ = run_to_goal(curr_goal_, str(goal_qr_id_) + "/" + str(try_counter) + str(num_tries_ + 1))
            if success_:
                print("##############")
                print("SUCCESS")
                print("##############")
                get_infos(scan_output_)
                return True
            rospy.sleep(3.)
        print("Could not reach qr code {} with current offset settings".format(goal_qr_id_))
        try_counter += 1
    print("NOT ABLE TO REACH QR CODE {}!".format(goal_qr_id_))
    return False
