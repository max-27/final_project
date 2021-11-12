#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
import rospy
import numpy as np # For random numbers
import tf 
import time
import math
import geometry_msgs.msg
 
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
 
#Create callback. This is what happens when a new message is received
def obj_cal(msg):
    global pose #to use it outside the callback
    pose = msg.pose
    br = tf.TransformBroadcaster() #publish/create frame using pose info
    # send out the pose of the turtle in the form of a transform
    br.sendTransform([pose.position.x, pose.position.y, pose.position.z],
                     [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                     rospy.Time.now(),
                     "qr_frame",
                     "camera_optical_link")
def code_cal(msg):
    #import pdb; pdb.set_trace()
    global code 
    code = msg.data #to get output from the qr code
    if code is not '': #split the output & use it afterwards
        fir = code.split("\r\n")
        global sec
        sec=[]
        for i,e in enumerate(fir):
            sec.append(e.split("="))

        global number_qr    
        number_qr = sec[-2][1]

# Initialize node
rospy.init_node('obj_pose')
 
#Initialize publisher
rospy.Subscriber('visp_auto_tracker/code_message', String , code_cal)
 
#Initialize publisher
rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, obj_cal)

listener = tf.TransformListener()   #to get translation & rotations between the frames
pr = tf.TransformBroadcaster() #to publish new frames

trs ={} #dictionary for storing all info from the qr

code = ''
d = [] #list for accessing specific qr

while not rospy.is_shutdown():   
    if code is not '':

        try:
            (trans1,rot1) = listener.lookupTransform('/map', '/qr_frame', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #Use qr_code to map frame translations
        pr.sendTransform(trans1,
                     [0.0, 0.0, 0.0, 1.0],
                     rospy.Time.now(),
                     "tr_frame",
                     "map")
        qr_name = 'qr_%d' % float(number_qr)
        
        #just for troubleshooting when it doesn't work at times
            #delay(2)
            #try:
            #    (trans1,rot1) = listener.lookupTransform('/map', '/qr_frame', rospy.Time.now())
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    continue

        if qr_name not in trs:
            trs[qr_name] = [trans1, float(sec[0][1]), float(sec[1][1]), float(sec[2][1]), float(sec[3][1]), float(sec[4][1]), sec[5][1]]
            d.append(qr_name)  
            print('Bravo QR detected: ' + qr_name)

    if len(trs) == 2: #if you detect two qr codes       
        #name = trs.keys() #list of keys for the dictionary
        qr1_coords_world = trs[d[0]][0]
        qr2_coords_world = trs[d[1]][0]
        qr1_coords_qr = trs[d[0]][1:3]
        qr2_coords_qr = trs[d[1]][1:3]
        
        world_x = qr2_coords_world[0] - qr1_coords_world[0] #create vector
        world_y = qr2_coords_world[1] - qr1_coords_world[1]
        distance_w = math.sqrt(world_x ** 2 + world_y ** 2) #Modulus
        theta_w = math.acos(world_y / distance_w) #Angle of rotation
        if world_x < 0.0: theta_w = 2*math.pi - theta_w

        qr_x = qr2_coords_qr[0] - qr1_coords_qr[0]
        qr_y = qr2_coords_qr[1] - qr1_coords_qr[1]
        distance_qr = math.sqrt(qr_x ** 2 + qr_y ** 2) #Modulus
        theta_qr = math.acos(qr_y / distance_qr) #Angle of rotation
        if qr_x < 0.0: theta_qr = 2*math.pi - theta_qr

        final_theta = theta_qr - theta_w #rotation between qr codes frame and map frame 

        # to get zero rotation wrt qr codes frame
        pr.sendTransform([0.0, 0.0, 0.0],tf.transformations.quaternion_from_euler(0.0, 0.0, final_theta),
                     rospy.Time.now(),
                     "rr_frame",
                     "tr_frame")

        # translate to qr codes frame, Bingo
        pr.sendTransform([-qr2_coords_qr[0], -qr2_coords_qr[1], 0.0],tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                     rospy.Time.now(),
                     "qr_pre_frame",
                     "rr_frame")

        # now you need to save wrt something static, because it vanishes after you move
        try:
            (trans2,rot2) = listener.lookupTransform('/map', '/qr_pre_frame', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pr.sendTransform(trans2,rot2,
                     rospy.Time.now(),
                     "qr_coordinate_frame",
                     "map")

        print(final_theta)
        #import pdb; pdb.set_trace()

rospy.spin()
