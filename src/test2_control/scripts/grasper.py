#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import time
from enum import Enum
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import os

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
    # global shoulder_pan_pub, shoulder_lift_pub, elbow_pub, wrist1_pub, wrist2_pub, wrist3_pub
    # global H1_F1J2_pub, H1_F1J3_pub, H1_F2J2_pub, H1_F2J3_pub, H1_F3J2_pub, H1_F3J3_pub, rate
    # shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10)
    # shoulder_lift_pub = rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10)
    # elbow_pub = rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10)
    # wrist1_pub = rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10)
    # wrist2_pub = rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10)
    # wrist3_pub = rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10)
    # H1_F1J2_pub = rospy.Publisher('/H1_F1J2_joint_position_controller/command', Float64, queue_size=10)
    # H1_F1J3_pub = rospy.Publisher('/H1_F1J3_joint_position_controller/command', Float64, queue_size=10)
    # H1_F2J2_pub = rospy.Publisher('/H1_F2J2_joint_position_controller/command', Float64, queue_size=10)
    # H1_F2J3_pub = rospy.Publisher('/H1_F2J3_joint_position_controller/command', Float64, queue_size=10)
    # H1_F3J2_pub = rospy.Publisher('/H1_F3J2_joint_position_controller/command', Float64, queue_size=10)
    # H1_F3J3_pub = rospy.Publisher('/H1_F3J3_joint_position_controller/command', Float64, queue_size=10)
    # rospy.init_node('supreme_commander', anonymous=True)
    # rate = rospy.Rate(1000) # 10hz

    rospy.init_node('supreme_commander')    

    global attach_srv, detach_srv
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")



    
def move(joint, position):
    if(joint == Joint.SHOULDER_PAN):
        os.system('rostopic pub -1 /shoulder_pan_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.SHOULDER_LIFT):
        os.system('rostopic pub -1 /shoulder_lift_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.ELBOW):
        os.system('rostopic pub -1 /elbow_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.WRIST1):
        os.system('rostopic pub -1 /wrist_1_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.WRIST2):
        os.system('rostopic pub -1 /wrist_2_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.WRIST3):
        os.system('rostopic pub -1 /wrist_3_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.H1_F1J2):
        os.system('rostopic pub -1 /H1_F1J2_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.H1_F1J3):
        os.system('rostopic pub -1 /H1_F1J3_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.H1_F2J2):
        os.system('rostopic pub -1 /H1_F2J2_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.H1_F2J3):
        os.system('rostopic pub -1 /H1_F2J3_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.H1_F3J2):
        os.system('rostopic pub -1 /H1_F3J2_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')
    elif(joint == Joint.H1_F3J3):
        os.system('rostopic pub -1 /H1_F3J3_joint_position_controller/command std_msgs/Float64 "data: '+str(position)+'" &')

    # time.sleep(0.2)


def reset():
    move(Joint.SHOULDER_LIFT, -0.785)
    move(Joint.ELBOW, 0.785)
    move(Joint.WRIST1, -1.57)
    move(Joint.WRIST2, -1.57)

def open_gripper():
    move(Joint.H1_F1J2, -0.3)
    move(Joint.H1_F1J3, 0)
    move(Joint.H1_F2J2, -0.3)
    move(Joint.H1_F2J3, 0)
    move(Joint.H1_F3J2, -0.3)
    move(Joint.H1_F3J3, 0)

def close_gripper():
    move(Joint.H1_F1J2, 0.25)
    move(Joint.H1_F1J3, 0.4)
    move(Joint.H1_F2J2, 0.25)
    move(Joint.H1_F2J3, 0.4)
    move(Joint.H1_F3J2, 0.25)
    move(Joint.H1_F3J3, 0.4)

def attach_joints():

    # Link them
    rospy.loginfo("Attaching wrist3 and box")
    req = AttachRequest()
    req.model_name_1 = "grasper"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = "box"
    req.link_name_2 = "link"

    attach_srv.call(req)

def detach_joints():
    
    # Link them
    rospy.loginfo("Detaching wrist3 and box")
    req = AttachRequest()
    req.model_name_1 = "grasper"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = "box"
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
    elif(cmd == "open"):
        open_gripper()
        detach_joints()
    elif(cmd == "close"):
        close_gripper()
        attach_joints()
    elif(cmd == "reset"):
        reset()


def main():


    while not rospy.is_shutdown():
        cmd = input()
        command(cmd)
        


if __name__ == '__main__':
    try:
        init()
        
        main()
    except rospy.ROSInterruptException:
        pass