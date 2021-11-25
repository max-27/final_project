#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Scan:

    def __init__(self):
        self.object_position = None
        self.code_message = None
        self.qr_dict = {}
        self.num_qr = None
        rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, self._scan_qr_pos_callback)
        rospy.Subscriber('/visp_auto_tracker/code_message', String, self._scan_qr_message_callback)

    def _scan_qr_pos_callback(self, msg):
        self.object_position = msg.pose

    def _scan_qr_message_callback(self, msg):
        # message looks like this:
        # <data: "X=2.67\r\nY=3.23\r\nX_next=0.1\r\nY_next=3.5\r\nN=2\r\nL=M">
        self.code_message = msg.data
        


    def validate_scan(self):
        print('Validating scan, z:{}'.format(self.object_position.position.z))
        if len(self.code_message)>2 and (self.object_position.position.z < 1.5):
            print(self.object_position.position.z)
            return True
        else:
            return False


    def scan(self, search_id="", specific_search=False):
        # next_qr_pos = rospy.wait_for_message("/visp_auto_tracker/object_position", PoseStamped)
        # qr_message = rospy.wait_for_message('/visp_auto_tracker/code_message', String)
        try:
            if self.object_position is not None and self.code_message is not None:
                position = self.object_position.position
                orientation = self.object_position.orientation
                pos_x, pos_y, self.pos_z = position.x, position.y, position.z
                orien_x, orien_y, orien_z, orien_w = orientation.x, orientation.y, orientation.z, orientation.w

                if pos_x + pos_y + self.pos_z + orien_x + orien_y + orien_z + orien_w != 1:
                    if pos_x != 0. and pos_y != 0. and self.pos_z != 0. and self.code_message != "":
                        try:
                            msg_list = [i.rsplit("=", 1)[1] for i in self.code_message.split("\r\n")]
                        except TypeError as e:
                            print(self.code_message.split("\r\n"))
                            raise e
                        curr_qr_pos = [float(msg_list[0]), float(msg_list[1])]
                        next_qr_pos = [float(msg_list[2]), float(msg_list[3])]
                        self.num_qr = float(msg_list[4])
                        msg_qr = msg_list[5]

                        # Adding the scanned QR code to a dict to check that we are not scanning the same QR code over again
                        if (self.num_qr not in self.qr_dict.keys() and specific_search is False) or (
                                specific_search and search_id == self.num_qr):
                            self.qr_dict[self.num_qr] = True
                            print(self.qr_dict)
                            if specific_search:
                                print("Found specific qr code")
                            return [[curr_qr_pos, next_qr_pos, self.num_qr, msg_qr], [position, orientation]]
                        else:
                            # print('QR code already scaned.')
                            return None

                else:
                    return None
                self.object_position = None
                self.code_message = None
        except IndexError:
            pass

   