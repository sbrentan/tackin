#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import time
from enum import Enum
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import os
import numpy as np
import torch
import cv2
from cv_bridge import CvBridge
import sys

from datetime import datetime


from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image


import kinematics as kin

mat = np.matrix

blocks = [ 'X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y2-Z2', 'X1-Y3-Z2-FILLET', 
         'X1-Y3-Z2', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2-FILLET', 'X2-Y2-Z2' ] # class names


class Joint(Enum):

    SHOULDER_PAN = 0
    SHOULDER_LIFT = 1
    ELBOW = 2
    WRIST1 = 3
    WRIST2 = 4
    WRIST3 = 5
    H1_F1J2 = 6
    H1_F1J3 = 7
    H1_F2J2 = 8
    H1_F2J3 = 9
    H1_F3J2 = 10
    H1_F3J3 = 11

def init():
    rospy.init_node('supreme_commander')

    global shoulder_pan_pub, shoulder_lift_pub, elbow_pub, wrist1_pub, wrist2_pub, wrist3_pub
    global H1_F1J2_pub, H1_F1J3_pub, H1_F2J2_pub, H1_F2J3_pub, H1_F3J2_pub, H1_F3J3_pub, rate
    shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10)
    shoulder_lift_pub = rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10)
    elbow_pub = rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10)
    wrist1_pub = rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10)
    wrist2_pub = rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10)
    wrist3_pub = rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10)
    H1_F1J2_pub = rospy.Publisher('/H1_F1J2_joint_position_controller/command', Float64, queue_size=10)
    H1_F1J3_pub = rospy.Publisher('/H1_F1J3_joint_position_controller/command', Float64, queue_size=10)
    H1_F2J2_pub = rospy.Publisher('/H1_F2J2_joint_position_controller/command', Float64, queue_size=10)
    H1_F2J3_pub = rospy.Publisher('/H1_F2J3_joint_position_controller/command', Float64, queue_size=10)
    H1_F3J2_pub = rospy.Publisher('/H1_F3J2_joint_position_controller/command', Float64, queue_size=10)
    H1_F3J3_pub = rospy.Publisher('/H1_F3J3_joint_position_controller/command', Float64, queue_size=10)

    # rospy.init_node('supreme_commander', anonymous=True)
    # rate = rospy.Rate(1000) # 10hz

    global attach_srv, detach_srv
    #rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    #rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    time.sleep(1)
    rospy.loginfo("Created all the publishers")

    rospy.Subscriber("/laser/scan", LaserScan, scan_callback)
    rospy.Subscriber("/camera/image_raw", Image, camera_callback)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)


def joint_state_callback(msg):

    global joint_states
    joint_states = []
    joint_states.append(msg.position[msg.name.index("shoulder_pan_joint")])
    joint_states.append(msg.position[msg.name.index("shoulder_lift_joint")])
    joint_states.append(msg.position[msg.name.index("elbow_joint")])
    joint_states.append(msg.position[msg.name.index("wrist_1_joint")])
    joint_states.append(msg.position[msg.name.index("wrist_2_joint")])
    joint_states.append(msg.position[msg.name.index("wrist_3_joint")])
    joint_states.append(msg.position[msg.name.index("wrist_3_joint")])
    joint_states.append(msg.position[msg.name.index("H1_F1J2")])
    joint_states.append(msg.position[msg.name.index("H1_F1J3")])
    joint_states.append(msg.position[msg.name.index("H1_F2J2")])
    joint_states.append(msg.position[msg.name.index("H1_F2J3")])
    joint_states.append(msg.position[msg.name.index("H1_F3J2")])
    joint_states.append(msg.position[msg.name.index("H1_F3J3")])


def camera_callback(rgb_msg):
   global last_image
   last_image = rgb_msg


def scan_callback(msg):
    global depth_ranges, nlasers
    depth_ranges = msg.ranges
    nlasers = len(msg.ranges)

def until_in_range(pos):
    for i in range(40):
        in_position = True
        for k, v in pos.items():
            print(joint_states[k])
            if(joint_states[k] <= v + 0.02 and joint_states[k] >= v - 0.02):
                in_position = False
                break;
        if(in_position): break
        time.sleep(0.1)



def move(joint, position):
    if(joint == Joint.SHOULDER_PAN):
        shoulder_pan_pub.publish(position)
    elif(joint == Joint.SHOULDER_LIFT):
        shoulder_lift_pub.publish(position)
    elif(joint == Joint.ELBOW):
        elbow_pub.publish(position)
    elif(joint == Joint.WRIST1):
        wrist1_pub.publish(position)
    elif(joint == Joint.WRIST2):
        wrist2_pub.publish(position)
    elif(joint == Joint.WRIST3):
        wrist3_pub.publish(position)
    elif(joint == Joint.H1_F1J2):
        H1_F1J2_pub.publish(position)
    elif(joint == Joint.H1_F1J3):
        H1_F1J3_pub.publish(position)
    elif(joint == Joint.H1_F2J2):
        H1_F2J2_pub.publish(position)
    elif(joint == Joint.H1_F2J3):
        H1_F2J3_pub.publish(position)
    elif(joint == Joint.H1_F3J2):
        H1_F3J2_pub.publish(position)
    elif(joint == Joint.H1_F3J3):
        H1_F3J3_pub.publish(position)

def reset():
    move(Joint.SHOULDER_PAN, 0)
    move(Joint.SHOULDER_LIFT, -0.785)
    move(Joint.ELBOW, 0.785)
    move(Joint.WRIST1, -1.57)
    move(Joint.WRIST2, -1.57)

def open_gripper():
    move(Joint.H1_F1J2, -0.4)
    move(Joint.H1_F1J3, 0)
    move(Joint.H1_F2J2, -0.4)
    move(Joint.H1_F2J3, 0)
    move(Joint.H1_F3J2, -0.4)
    move(Joint.H1_F3J3, 0)

def close_gripper():
    move(Joint.H1_F1J2, 0.25)
    move(Joint.H1_F1J3, 0.4)
    move(Joint.H1_F2J2, 0.25)
    move(Joint.H1_F2J3, 0.4)
    move(Joint.H1_F3J2, 0.25)
    move(Joint.H1_F3J3, 0.4)

def attach_joints(box):

    # Link them
    rospy.loginfo("Attaching wrist3 and box")
    req = AttachRequest()
    req.model_name_1 = "grasper"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = box
    req.link_name_2 = "link"

    attach_srv.call(req)

def detach_joints(box):

    # Link them
    rospy.loginfo("Detaching wrist3 and box")
    req = AttachRequest()
    req.model_name_1 = "grasper"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = box
    req.link_name_2 = "link"

    detach_srv.call(req)

def command(cmd):
    if(cmd == "rise"):
        move(Joint.SHOULDER_LIFT, -0.785)
        time.sleep(0.1)
        move(Joint.ELBOW, 0.785)
    elif(cmd == "descend"):
        move(Joint.ELBOW, 0)
        time.sleep(0.1)
        move(Joint.SHOULDER_LIFT, 0)
        time.sleep(0.3)
    elif(cmd == "release"):
        move(Joint.SHOULDER_PAN, 1.57)
        time.sleep(0.1)
        command("descend")
        command("open")
    elif(cmd.split()[0] == "open"):
        open_gripper()
        time.sleep(1.5)
        if(len(cmd.split()) > 1):
            detach_joints(cmd.split()[1])
    elif(cmd.split()[0] == "close"):
        close_gripper()
        time.sleep(1.5)
        if(len(cmd.split()) > 1):
            attach_joints(cmd.split()[1])
    elif(cmd[0:3] == "kin"):
        compute_kinematik(cmd.split()[1:])
    elif(cmd == "depth"):
        index_min = np.argmin(depth_ranges)
        angle = (index_min * 90 / nlasers)
        rad_angle = np.deg2rad(angle)
        xdist = math.cos(rad_angle) * depth_ranges[index_min]
        ydist = math.sin(rad_angle) * depth_ranges[index_min]
        mode = 2
        if(xdist <= 0.5 or ydist <= 0.5):
            mode = 0

        thetas = compute_kinematik([mode, xdist, ydist, -0.2])

        print("Current Time = ", datetime.now().strftime("%H:%M:%S"))

        until_in_range({
                0  : thetas[0],
                1 : thetas[1],
                2         : thetas[2],
                3        : thetas[3],
                4        : thetas[4],
                5        : thetas[5],
            })

        print("Current Time = ", datetime.now().strftime("%H:%M:%S"))

        compute_kinematik([mode, xdist, ydist])

    elif(cmd == "camera"):
        camera_image = CvBridge().imgmsg_to_cv2(last_image)
        cv2.imwrite("cool_camera_image.jpg", camera_image)

    elif(cmd.split()[0] == "spawnbox"):
        x, y, name = cmd.split()[1:]
        os.system("roslaunch test2_gazebo spawn_box.launch x:="+x+" y:="+y+" name:="+name+" > /dev/null")
        print("Box spawned")

    elif(cmd == "detect"):
        model = torch.hub.load('src/yolov5', 'custom', path="best.pt", source="local", device="cpu")
        camera_image = CvBridge().imgmsg_to_cv2(last_image)
        results = model(camera_image)
        print(blocks[results.pandas().xyxy[0].at[0, "class"]])

    elif(cmd == "reset"):
        reset()

    elif(cmd == "x"):
        sys.exit()

def compute_kinematik(args): #BEST ARGS[0] = 6
    args[0] = int(args[0])
    args[1] = float(args[1]) + 0.015
    args[2] = float(args[2]) + 0.015
    zposition = -0.38
    if(len(args) > 3):
        zposition = float(args[3])

    print(args)
    thetas = kin.invKine((mat([
        [1, 0, 0, -args[1]],
        [0, -1, 0, -args[2]],
        [0, 0, -1, zposition],
        [0, 0, 0, 1]
             ])))
    print(thetas[0,args[0]], thetas[1,args[0]], thetas[2,args[0]], thetas[3,args[0]], thetas[4,args[0]], thetas[5,args[0]])

    move(Joint.WRIST1, thetas[3,args[0]])
    move(Joint.WRIST2, thetas[4,args[0]])
    move(Joint.WRIST3, thetas[5,args[0]])

    move(Joint.SHOULDER_PAN, thetas[0,args[0]])
    move(Joint.SHOULDER_LIFT, thetas[1,args[0]])
    move(Joint.ELBOW, thetas[2,args[0]])
    
    return thetas


def main():


    while not rospy.is_shutdown():
        print("Insert command: ")
        cmd = input()
        command(cmd)



if __name__ == '__main__':
    try:
        init()

        main()
    except rospy.ROSInterruptException:
        pass
