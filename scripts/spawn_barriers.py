#!/usr/bin/env python

import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_barriers")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)


    obstacle = rospy.get_param('~obstacle')
    with open(obstacle, "r") as f:
        obstacle_product_xml = f.read()

    barrier_poses = [[-4.5, -1.5],
                     [-5.6, -0.5],
                     [-3.3, 1.3],
                     [-5.16, 1.6],
                     [-1.8, 0.9],
                     [4,0],
                     [5.5, 0.5],
                     [5.5, -1.66]]

    for i, pos in enumerate(barrier_poses):
        orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, np.random.uniform(-np.pi/2., np.pi/2)))
        pos_noise = np.random.uniform(-0.2, 0.2,size=2)
        item_pose = Pose(Point(x=pos[0]+pos_noise[0],
                               y=pos[1]+pos_noise[0],
                               z=0),
                               orient)

        spawn_model("obstacle_{}".format(i), obstacle_product_xml, "", item_pose, "world")
