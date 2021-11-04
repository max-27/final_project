#!/usr/bin/env python

import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_markers")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)


    markers = rospy.get_param('~markers')
    with open(markers, "r") as f:
        markers_product_xml = f.read()

    marker_poses = [[-3.24, 0, 1.57075],
                    [4, -0.1, -1.57075],
                    [-6.5, 0, -1.57075],
                    [-3.658665, 0.511559, -3.137797]] # x, y, rot_z

    layout = rospy.get_param('~layout') - 1 # align for list index, -> 0 for random layout
    rospy.loginfo("The Layout is {}".format(layout))

    layout = 0
    if layout == -1:
        layout = random.randint(0,len(marker_poses))

    orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, marker_poses[layout][2]))
    item_pose = Pose(Point(x=marker_poses[layout][0], y=marker_poses[layout][1], z=0),   orient)
    spawn_model("Markers", markers_product_xml, "", item_pose, "world")
