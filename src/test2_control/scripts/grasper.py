#!/usr/bin/env python
# license removed for brevity
import rospy
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
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from datetime import datetime


from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image


import kinematics as kin
import recognizer as rec

mat = np.matrix

posx = 0
posy = 0


blocks = [ 'X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y2-Z2', 'X1-Y3-Z2-FILLET', 
         'X1-Y3-Z2', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2-FILLET', 'X2-Y2-Z2' ] # class names


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

    global attach_srv, detach_srv, last_updated_image
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # camera_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    # camera_srv.wait_for_service()
    # rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    time.sleep(1)

    last_updated_image = 0

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
    joint_states.append(msg.position[msg.name.index("H1_F1J2")])
    joint_states.append(msg.position[msg.name.index("H1_F1J3")])
    joint_states.append(msg.position[msg.name.index("H1_F2J2")])
    joint_states.append(msg.position[msg.name.index("H1_F2J3")])
    joint_states.append(msg.position[msg.name.index("H1_F3J2")])
    joint_states.append(msg.position[msg.name.index("H1_F3J3")])


def camera_callback(rgb_msg):
   global last_image, last_updated_image
   last_image = rgb_msg
   last_updated_image += 1
   # print("Updated image")


def scan_callback(msg):
    global depth_ranges, nlasers
    depth_ranges = msg.ranges
    nlasers = len(msg.ranges)

def until_in_range(pos, max_wait = 8):
    if type(pos) is not dict:
        print("not dict")
        newpos = {
            SHOULDER_PAN : pos[0],
            SHOULDER_LIFT: pos[1],
            ELBOW        : pos[2],
            WRIST1       : pos[3],
            WRIST2       : pos[4]
        }
        pos = newpos

    for i in range(max_wait * 10):
        in_position = True
        for k, v in pos.items():
            if(not (joint_states[k] <= v + 0.05 and joint_states[k] >= v - 0.05)):
                in_position = False
                break;
        if(in_position): break
        time.sleep(0.1)


def rotate_wrist(angle, mode = 0, wait = True): #mode:0 for absolute position, mode:1 for relative position
    if(mode == 1):
        angle += joint_states[WRIST3]

    if(angle > np.pi):
        angle -= np.pi * 2
    elif(angle < -np.pi):
        angle += np.pi * 2

    move(WRIST3, angle)

    if(wait):
        until_in_range({WRIST3 : angle})


def get_object_class():
    image = CvBridge().imgmsg_to_cv2(last_image)
    results = rec.getClass(image)
    

    max1 = 0
    max2 = 0
    max3 = 0
    max4 = 0
    max5 = 0
    max6 = 0
    for k, ar in results.items():
        print(k+" -> "+str(ar))
        max1 = max(ar[0], max1)
        max2 = max(ar[1], max2)
        max3 = max(ar[2], max3)
        max4 = max(ar[3], max4)
        max5 = min(ar[4], max5)
        max6 = min(ar[5], max6)

    for k, ar in results.items():
        val1 = ar[0]/max1
        val2 = ar[1]/max2
        val3 = ar[2]/max3
        val4 = ar[3]/max4
        val5 = max5/ar[4]
        val6 = max6/ar[5]
        avg = (val1 * 0.15) + (val2 * 0.4) + (val3 * 0.15) + (val4 * 0.1) + (val5 * 0.1) + (val6 * 0.1)
        print(k+' -> '+str(avg))



def move(joint, position):
    if(joint == SHOULDER_PAN):
        shoulder_pan_pub.publish(position)
    elif(joint == SHOULDER_LIFT):
        shoulder_lift_pub.publish(position)
    elif(joint == ELBOW):
        elbow_pub.publish(position)
    elif(joint == WRIST1):
        wrist1_pub.publish(position)
    elif(joint == WRIST2):
        wrist2_pub.publish(position)
    elif(joint == WRIST3):
        wrist3_pub.publish(position)
    elif(joint == H1_F1J2):
        H1_F1J2_pub.publish(position)
    elif(joint == H1_F1J3):
        H1_F1J3_pub.publish(position)
    elif(joint == H1_F2J2):
        H1_F2J2_pub.publish(position)
    elif(joint == H1_F2J3):
        H1_F2J3_pub.publish(position)
    elif(joint == H1_F3J2):
        H1_F3J2_pub.publish(position)
    elif(joint == H1_F3J3):
        H1_F3J3_pub.publish(position)

def reset():
    move(SHOULDER_PAN, 0)
    move(SHOULDER_LIFT, -0.785)
    move(ELBOW, 0.785)
    move(WRIST1, -1.57)
    move(WRIST2, -1.57)

def open_gripper():
    move(H1_F1J2, -0.4)
    move(H1_F1J3, 0)
    move(H1_F2J2, -0.4)
    move(H1_F2J3, 0)
    move(H1_F3J2, -0.4)
    move(H1_F3J3, 0)
    until_in_range({
        H1_F1J2  : -0.4,
        H1_F1J3  : 0,
        H1_F2J2  : -0.4,
        H1_F2J3  : 0,
        H1_F3J2  : -0.4,
        H1_F3J3  : 0,
    })


def close_gripper():
    move(H1_F1J2, 0.25)
    move(H1_F1J3, 0.4)
    move(H1_F2J2, 0)
    move(H1_F2J3, 0)
    move(H1_F3J2, 0.25)
    move(H1_F3J3, 0.4)
    # print("Current Time = ", datetime.now().strftime("%H:%M:%S"))
    until_in_range({
        H1_F1J2  : 0.25,
        H1_F1J3  : 0.4,
        H1_F2J2  : 0,
        H1_F2J3  : 0,
        H1_F3J2  : 0.25,
        H1_F3J3  : 0.4,
    })

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

def set_joint_states(arr):
    for i in range(len(arr)):
        move(i, arr[i])

def command(cmd):
    if(len(cmd.split()) < 1):
        return
    # elif(cmd == "release"):

    elif(cmd.split()[0] == "high"):
        mode = 2
        if(len(cmd.split()) > 1): mode = cmd.split()[1]
        thetas = compute_kinematik([mode, posx, posy, -0.2], True, True, 2)

    elif(cmd.split()[0] == "low"):
        mode = 2
        if(len(cmd.split()) > 1): mode = cmd.split()[1]
        thetas = compute_kinematik([mode, posx, posy, -0.3], True, True, 2)
        thetas = compute_kinematik([mode, posx, posy], True, True, 2)


    elif(cmd.split()[0] == "open"):
        open_gripper()
        if(len(cmd.split()) > 1):
            detach_joints(cmd.split()[1])

    elif(cmd.split()[0] == "close"):
        if(len(cmd.split()) > 1):
            attach_joints(cmd.split()[1])
        close_gripper()
        
    elif(cmd[0:3] == "kin"):
        compute_kinematik(cmd.split()[1:])

    elif(cmd == "depth"):
        open_gripper()

        mode = 2

        command("dist")

        time.sleep(2)

        image = CvBridge().imgmsg_to_cv2(last_image)
        cv2.imwrite("pre_image.jpg", image)
        angle, pose1 = rec.getPose(image)

        xdiff = pose1[0]
        ydiff = pose1[1]
        xdiff = xdiff * 0.143 / 418
        ydiff = ydiff * 0.046 / 134
        print(angle, xdiff, ydiff, pose1)
        compute_kinematik([mode, posx - xdiff, posy + ydiff, -0.2], True)

        # time.sleep(2)
        # image = CvBridge().imgmsg_to_cv2(last_image)
        # cv2.imwrite("mid_image.jpg", image)

        print(angle)
        angle = np.abs(angle)
        angle = np.deg2rad(angle)
        angle = round(angle, 2)
        print(angle)
        print(joint_states[WRIST3])
        rotate_wrist(angle, 1)
        
        time.sleep(2)

        get_object_class()
        

    elif(cmd == "dist"):
        index_min = np.argmin(depth_ranges)
        angle = (index_min * 180 / nlasers)
        rad_angle = np.deg2rad(angle)
        ydist = math.cos(rad_angle) * depth_ranges[index_min]
        xdist = math.sin(rad_angle) * depth_ranges[index_min]

        print("Found position: ", xdist, ydist)

        jump = 0.05

        left = index_min - 1
        dr = depth_ranges
        while(left >= 0):
            print(dr[left + 1], dr[left])
            if(np.abs(dr[left + 1] - dr[left]) > jump):
                break
            left -= 1

        print('------')
        right = index_min + 1
        dr = depth_ranges
        while(right < len(dr)):
            print(dr[right - 1], dr[right])
            if(np.abs(dr[right - 1] - dr[right]) > jump):
                break
            right += 1

        left += 1
        right -= 1
        print(left, right)

        angle = (left * 180 / nlasers)
        rad_angle = np.deg2rad(angle)
        yleft = math.cos(rad_angle) * depth_ranges[left]
        xleft = math.sin(rad_angle) * depth_ranges[left]

        # print(angle, xleft, yleft)

        angle = (right * 180 / nlasers)
        rad_angle = np.deg2rad(angle)
        yright = math.cos(rad_angle) * depth_ranges[right]
        xright = math.sin(rad_angle) * depth_ranges[right]

        # print(angle, xright, yright)

        finalx = ((xleft + xright - xdist) - xdist)/2 + xdist
        finaly = ((yleft + yright - ydist) - ydist)/2 + ydist
        print(finalx, finaly)

        mode = 2
        thetas = compute_kinematik([mode, finaly, finalx, -0.2])


    elif(cmd == "camera"):
        camera_image = CvBridge().imgmsg_to_cv2(last_image)
        cv2.imwrite("cool_camera_image.jpg", camera_image)

    elif(cmd.split()[0] == "spawnbox"):
        x, y, name = cmd.split()[1:]
        os.system("roslaunch test2_gazebo spawn_box.launch x:="+x+" y:="+y+" name:="+name+" > /dev/null")

    elif(cmd.split()[0] == "spawnbrick"):
        x, y, name, model = cmd.split()[1:]
        os.system("roslaunch test2_gazebo spawn_brick.launch x:="+x+" y:="+y+" name:="+name+" model:="+model+" > /dev/null")        

    elif(cmd == "detect"):
        # model = torch.hub.load('src/yolov5', 'custom', path="best.pt", source="local", device="cpu", pretrained=True)
        model = torch.hub.load('src/yolov5', 'custom', path="best.pt", source="local", device="cpu")
        camera_image = CvBridge().imgmsg_to_cv2(last_image)
        results = model(camera_image)
        if(results.pandas().xyxy[0].empty):
            print("Nothing found")
        else:
            print(results.pandas().xyxy)
            print(blocks[results.pandas().xyxy[0].at[0, "class"]])
            print(results.pandas().xyxy[0].at[0, "confidence"])

    elif(cmd == "test"):
        print(joint_states)
        print(joint_states[1])

    elif(cmd.split()[0] == "rotate"):

        rotate_wrist(float(cmd.split()[2]), int(cmd.split()[1]))
        # if(cmd.split()[1] == "0"):
        #     move(WRIST3, float(cmd.split()[2]))
        # elif(cmd.split()[1] == "1"):
        #     move(WRIST3, joint_states[WRIST3] + float(cmd.split()[2]))

    elif(cmd == "temp"):
        get_object_class()


    elif(cmd == "reset"):
        reset()


    elif(cmd == "dioorso"):
        set_joint_states([0.6420331459112516, -0.44856000332471435, 2.028709678559421, 2.632259060910691, -1.5708015714139925, 
            2.212829601314244, 2.212829601314244, -0.3999997928324186, 5.325995622307289e-06, -0.3999989075042878, 
            -3.6077062857131637e-06, -0.4000011540877457, -2.423872540902039e-06])


    elif(cmd == "x"):
        sys.exit()

def compute_kinematik(args, ignorew3 = False, wait = True, max_wait = 8): #BEST ARGS[0] = 6
    mode = int(args[0])
    x = float(args[1])
    y = float(args[2])
    zposition = -0.38
    if(len(args) > 3):
        zposition = float(args[3])

    global posx, posy
    posx = x
    posy = y

    # print(args)
    thetas = kin.invKine((mat([
        [1, 0, 0, -x],
        [0, -1, 0, -y],
        [0, 0, -1, zposition],
        [0, 0, 0, 1]
             ])))

    move(WRIST1, thetas[3,mode])
    move(WRIST2, thetas[4,mode])

    w3rot = thetas[SHOULDER_PAN,mode] + 1.57
    if(not ignorew3):
        rotate_wrist(w3rot, 0, False)

    move(SHOULDER_PAN, thetas[0,mode])
    move(SHOULDER_LIFT, thetas[1,mode])
    move(ELBOW, thetas[2,mode])

    if(wait):
        until_in_range({
            SHOULDER_PAN  : thetas[SHOULDER_PAN, mode],
            SHOULDER_LIFT : thetas[SHOULDER_LIFT, mode],
            ELBOW         : thetas[ELBOW, mode],
            WRIST1        : thetas[WRIST1, mode],
            WRIST2        : thetas[WRIST2, mode],
            WRIST3        : w3rot,
        }, max_wait)
    
    # map thetas
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
