#!/usr/bin/env python3

import rospy, rospkg
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.srv import GetModelState, SetModelState, DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState 
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

import numpy as np
import torch
import sys
import math
import signal
import tf
import threading

import cv2
from cv_bridge import CvBridge
from scipy.spatial import distance

import time
from datetime import datetime

import kinematics as kin
import recognizer as rec
import spawner    as spa


BRICK_HEIGHT_1 = 0.0565
BRICK_HEIGHT_2 = 0.0855

BRICK_WIDTH_1 = 0.046
BRICK_WIDTH_2 = 0.095
BRICK_WIDTH_3 = 0.143
BRICK_WIDTH_4 = 0.191

NO_COLLISION_2 = 0.05615
NO_COLLISION_1 = 0.02715

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

H1_F1J1 = 12
H1_F2J1 = 13
H1_F3J1 = 14


KPI_11 = 0
KPI_12 = 0
KPI_21 = 0


BLOCKS = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 
          'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

BLOCKS_HEIGHT = [NO_COLLISION_2,NO_COLLISION_1,NO_COLLISION_2,NO_COLLISION_2,NO_COLLISION_2,NO_COLLISION_2,
                 NO_COLLISION_2,NO_COLLISION_1,NO_COLLISION_2,NO_COLLISION_2,NO_COLLISION_2]

BLOCKS_WIDTH = [BRICK_WIDTH_1, BRICK_WIDTH_2, BRICK_WIDTH_2, BRICK_WIDTH_2, BRICK_WIDTH_2, BRICK_WIDTH_3, 
                BRICK_WIDTH_3, BRICK_WIDTH_4, BRICK_WIDTH_4, BRICK_WIDTH_2, BRICK_WIDTH_2 ]

nblocks = np.zeros(11)
first_bricks = np.zeros(11, dtype=int)

mat = np.matrix

rp = rospkg.RosPack()
pkg_path = rp.get_path("tackin_control")

model_side = 0 #0 up, 1 down, 2 side
model_direction  = 0#0 up, 1 down
model_direction2 = 0#0 left, 1 right
hside = 0
pre_image = 0
last_hbrick = 0
brick_rot = 0
attached_model = -1
attached_brick = 0
posx = 0
posy = 0

brick_pixels_height = 0
brick_pixels_box = [0, 0, 0, 0]

BRICK_PIXEL_HEIGHT_1 = 150
BRICK_PIXEL_HEIGHT_2 = 200

def init():
    signal.signal(signal.SIGINT, kill)
    signal.signal(signal.SIGTERM, kill)

    rospy.init_node('supreme_commander')

    global shoulder_pan_pub, shoulder_lift_pub, elbow_pub, wrist1_pub, wrist2_pub, wrist3_pub
    global H1_F1J1_pub, H1_F1J2_pub, H1_F1J3_pub, H1_F2J1_pub, H1_F2J2_pub, H1_F2J3_pub, H1_F3J1_pub, H1_F3J2_pub, H1_F3J3_pub, rate
    shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10)
    shoulder_lift_pub = rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10)
    elbow_pub = rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10)
    wrist1_pub = rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10)
    wrist2_pub = rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10)
    wrist3_pub = rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10)
    H1_F1J1_pub = rospy.Publisher('/H1_F1J1_joint_position_controller/command', Float64, queue_size=10)
    H1_F1J2_pub = rospy.Publisher('/H1_F1J2_joint_position_controller/command', Float64, queue_size=10)
    H1_F1J3_pub = rospy.Publisher('/H1_F1J3_joint_position_controller/command', Float64, queue_size=10)
    H1_F2J1_pub = rospy.Publisher('/H1_F2J1_joint_position_controller/command', Float64, queue_size=10)
    H1_F2J2_pub = rospy.Publisher('/H1_F2J2_joint_position_controller/command', Float64, queue_size=10)
    H1_F2J3_pub = rospy.Publisher('/H1_F2J3_joint_position_controller/command', Float64, queue_size=10)
    H1_F3J1_pub = rospy.Publisher('/H1_F3J1_joint_position_controller/command', Float64, queue_size=10)
    H1_F3J2_pub = rospy.Publisher('/H1_F3J2_joint_position_controller/command', Float64, queue_size=10)
    H1_F3J3_pub = rospy.Publisher('/H1_F3J3_joint_position_controller/command', Float64, queue_size=10)

    global configuration, total_bricks
    global attach_srv, detach_srv, mstate_srv, last_updated_image, bridge, spawn_model, delete_model

    print("Starting services...")

    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()

    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()

    mstate_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    mstate_srv.wait_for_service()

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    rospy.wait_for_service("gazebo/delete_model")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    time.sleep(1)

    last_updated_image = 0

    rospy.Subscriber("/laser/scan", LaserScan, scan_callback)
    rospy.Subscriber("/camera/image_raw", Image, camera_callback)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.Subscriber('/camera/scan', LaserScan, depth_callback)

    bridge = CvBridge()

    attach("stand", "link", "ground_plane", "link")

def kill(_signo, _stack_frame):
    sys.exit(0)


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
    joint_states.append(msg.position[msg.name.index("H1_F1J1")])
    joint_states.append(msg.position[msg.name.index("H1_F2J1")])
    joint_states.append(msg.position[msg.name.index("H1_F3J1")])


def camera_callback(rgb_msg):
   global last_image, last_updated_image
   last_image = rgb_msg
   last_updated_image += 1


def scan_callback(msg):
    global depth_ranges, nlasers
    depth_ranges = msg.ranges
    nlasers = len(msg.ranges)

def depth_callback(msg):
    global hdist
    hdist = min(msg.ranges)

def until_in_range(pos, max_wait = 8):
    if type(pos) is not dict:
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
            if(not (joint_states[k] <= v + 0.04 and joint_states[k] >= v - 0.04)):
                in_position = False
                break;
        if(in_position): break
        time.sleep(0.1)


def rotate_wrist(angle, mode = 0, wait = True): #mode:0 for absolute position, mode:1 for relative position
    if(mode == 1):
        angle += joint_states[WRIST3]
    return rotate(WRIST3, angle, wait)

def rotate(joint, angle, wait = True):

    pre_angle = angle
    curr_rot = joint_states[joint]
    if(angle > np.pi * 2):
        angle -= np.pi * 2
    elif(angle < -np.pi * 2):
        angle += np.pi * 2
    else:
        if(angle > curr_rot and angle - curr_rot > np.pi*2 - (angle - curr_rot) ):
            new = curr_rot - (np.pi*2 - (angle - curr_rot))
            if(new > np.pi * 2):
                angle = new
        elif(angle < curr_rot and curr_rot - angle > np.pi*2 - (curr_rot - angle)):
            new = curr_rot + (np.pi*2 - (curr_rot - angle))
            if(new < np.pi * 2):
                angle = new

    move(joint, angle)

    if(wait):
        until_in_range({joint : angle})
    return angle

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

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
    elif(joint == H1_F1J1):
        H1_F1J1_pub.publish(position)
    elif(joint == H1_F1J2):
        H1_F1J2_pub.publish(position)
    elif(joint == H1_F1J3):
        H1_F1J3_pub.publish(position)
    elif(joint == H1_F2J1):
        H1_F2J1_pub.publish(position)
    elif(joint == H1_F2J2):
        H1_F2J2_pub.publish(position)
    elif(joint == H1_F2J3):
        H1_F2J3_pub.publish(position)
    elif(joint == H1_F3J1):
        H1_F3J1_pub.publish(position)
    elif(joint == H1_F3J2):
        H1_F3J2_pub.publish(position)
    elif(joint == H1_F3J3):
        H1_F3J3_pub.publish(position)

def open_gripper():
    move(H1_F1J1,  0.8)
    move(H1_F3J1, -0.8)

    move(H1_F1J2, -0.4)
    move(H1_F1J3,  0)
    move(H1_F2J2, -0.4)
    move(H1_F2J3,  0)
    move(H1_F3J2, -0.4)
    move(H1_F3J3,  0)
    until_in_range({
        H1_F1J1  :  0.8,
        H1_F3J1  : -0.8,
        H1_F1J2  : -0.4,
        H1_F1J3  :  0,
        H1_F2J2  : -0.4,
        H1_F2J3  :  0,
        H1_F3J2  : -0.4,
        H1_F3J3  :  0,
    }, 2)

def close_gripper(ignoref1 = False):

    global brick_pixels_height
    joint3 = 0.8
    joint2 = -0.36
    if(np.abs(brick_pixels_height - BRICK_PIXEL_HEIGHT_2) > np.abs(brick_pixels_height - BRICK_PIXEL_HEIGHT_1)):
        joint2 = -0.15

    if(not ignoref1):
        move(H1_F1J2, joint2)
        move(H1_F1J3, joint3)
    move(H1_F2J2, joint2)
    move(H1_F2J3, joint3)
    move(H1_F3J2, joint2)
    move(H1_F3J3, joint3)
    if(not ignoref1):
        until_in_range({
            H1_F1J2  : joint2,
            H1_F1J3  : joint3,
            H1_F2J2  : joint2,
            H1_F2J3  : joint3,
            H1_F3J2  : joint2,
            H1_F3J3  : joint3,
        }, 2)
    else:
        until_in_range({
            H1_F2J2  : joint2,
            H1_F2J3  : joint3,
            H1_F3J2  : joint2,
            H1_F3J3  : joint3,
        }, 2)

def attach(m1, l1, m2, l2):
    req = AttachRequest()
    req.model_name_1 = m1
    req.link_name_1 = l1
    req.model_name_2 = m2
    req.link_name_2 = l2

    attach_srv.call(req)

def detach(m1, l1, m2, l2):
    req = AttachRequest()
    req.model_name_1 = m1
    req.link_name_1 = l1
    req.model_name_2 = m2
    req.link_name_2 = l2

    detach_srv.call(req)

def attach_joints(nbrick):

    req = AttachRequest()
    req.model_name_1 = "grasper"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = "brick" + str(nbrick)
    req.link_name_2 = "link"

    attach_srv.call(req)
    global attached_brick
    attached_brick = nbrick

def attach_bricks(model):

    global first_bricks

    req = AttachRequest()
    req.model_name_1 = "brick"+str(attached_brick)
    req.link_name_1 = "link"
    req.model_name_2 = "brick" + str(first_bricks[model])
    req.link_name_2 = "link"

    attach_srv.call(req)

def detach_joints(name = ""):

    global attached_brick, attached_model, brick_rot
    nbrick = attached_brick

    if(len(name) == 0):
        name = "brick" + str(nbrick)
    
    req = AttachRequest()
    req.model_name_1 = "grasper"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = name
    req.link_name_2 = "link"

    detach_srv.call(req)
    attached_brick = 0
    attached_model = -1
    brick_rot = 0

def get_model_id():
    global total_bricks
    names = []
    for i in range(total_bricks):
        names.append("brick"+str(i+1))

    curr_pos = (posx, posy, 0)
    min_brick = 0
    min_dist = 100
    for name in names:
        mstate = mstate_srv.call(name, "")
        m_pos = (mstate.pose.position.x, mstate.pose.position.y, 0)
        d = distance.euclidean(curr_pos, m_pos)
        if(d < min_dist):
            min_dist = d
            min_brick = int(name[5:])

    return min_brick

def print_time():
    print("Current Time = ", datetime.now().strftime("%H:%M:%S"))

def set_joint_states(arr):
    for i in range(len(arr)):
        move(i, arr[i])

def detect(dataset = "dataset.pt", is_correction = False):
    global attached_model, brick_dist, last_hbrick, pre_image, hside, model_side, KPI_12

    brick_dist = hdist
    prev_attached_model = attached_model

    image = rec.filterImage(pre_image)
    image[np.all(image == (0, 0, 0), axis=-1)] = (130,130,130)

    model = torch.hub.load(pkg_path + "/yolov5", 'custom', path=pkg_path+"/"+dataset, source="local", device="cpu")
    model.cpu()
    results = model(image, size=500)
    if(results.pandas().xyxy[0].empty):
        print("Nothing found")
    else:
        # print(results.pandas().xyxy)


        model_side = 0
        if(dataset == "dataset.pt"):
            attached_model = int(results.pandas().xyxy[0].at[0, "class"]/3)
            model_side = results.pandas().xyxy[0].at[0, "class"]%3
        else:
            attached_model = int(results.pandas().xyxy[0].at[0, "class"])


        # hbrick = 0.32 - brick_dist
        if(model_side == 0 or model_side == 1):
            hbrick = last_hbrick
            if(np.abs(hbrick - BRICK_HEIGHT_1) > np.abs(hbrick - BRICK_HEIGHT_2)):
                corr_brick = BLOCKS[attached_model].replace("Z1", "Z2")
            else:
                corr_brick = BLOCKS[attached_model].replace("Z2", "Z1")

            if(corr_brick not in BLOCKS):
                print("Wrong detection, reattempting...")
                detect("best.pt", is_correction)
                return
            else:
                attached_model = BLOCKS.index(corr_brick)

        if(model_side == 1):
            if(np.abs(hside - BRICK_WIDTH_1) > np.abs(hside - BRICK_WIDTH_2)):
                if(np.abs(hside - BRICK_HEIGHT_2) > np.abs(hside - BRICK_WIDTH_2)):
                    corr_brick = BLOCKS[attached_model].replace("X1", "X2")
            else:
                if(np.abs(hside - BRICK_HEIGHT_1) > np.abs(hside - BRICK_WIDTH_1)):
                    corr_brick = BLOCKS[attached_model].replace("X2", "X1")

            if(corr_brick not in BLOCKS):
                print("Wrong detection, reattempting...")
                detect("best.pt", is_correction)
                return
            else:
                attached_model = BLOCKS.index(corr_brick)

        directions = ["UP", "DOWN", "SIDE"]

        if(is_correction and prev_attached_model != attached_model):
            print("Corrected brick to " + BLOCKS[attached_model])
        elif(not is_correction):
            print("Brick found: " + BLOCKS[attached_model] + " with direction " + directions[model_side])

    KPI_12 = datetime.now()


def adjust():
    global model_side, pre_image, last_hbrick, model_direction, model_direction2, attached_model

    down_pos = -0.19
    stand_y_original = -0.675
    redetect = False


    if(model_side == 2 or model_side == 1):#90/-90 or 180

        down_pos += 0.01
        brick_pixels_width = brick_pixels_box[1] - brick_pixels_box[0]
        brick_width = brick_pixels_width * 0.143 / 418
        stand_y = stand_y_original + ((0.1 - brick_width)/2)
        stand_y += 0.02

        compute_kinematik([2, 0, stand_y, down_pos + 0.18])#over stand
        rotate_wrist(np.pi/2, 1)

        if(model_side == 2 and BLOCKS[attached_model] == "X1-Y1-Z2"):
            if(model_direction2 == 0):
                rotate_wrist(np.pi, 1)
        elif(model_side == 2 and model_direction2 == 1):
            rotate_wrist(np.pi, 1)
            model_direction = np.abs(model_direction - 1)

        compute_kinematik([2, 0, stand_y, down_pos], True)#down
        detach("grasper", "wrist_3_link", "brick"+str(attached_brick), "link")
        attach("stand", "link", "brick"+str(attached_brick), "link")
        open_gripper()

        compute_kinematik([2, 0, stand_y, down_pos + 0.28])#up
        compute_kinematik([0, 0, stand_y, down_pos + 0.28, 2])#change rotation
        compute_kinematik([0, 0, -0.3, down_pos + 0.28, 2])#back
        compute_kinematik([0, 0, -0.3, down_pos, 2])#down

        rotate_wrist(1.57, 0)

        compute_kinematik([0, 0, -0.405, down_pos, 2], True)#forward
        detach("stand", "link", "brick"+str(attached_brick), "link")
        attach("grasper", "wrist_3_link", "brick"+str(attached_brick), "link")
        close_gripper()

        compute_kinematik([0, 0, -0.405, down_pos + 0.18, 2], True)#up

        if(BLOCKS[attached_model] == "X1-Y1-Z2" and model_side == 2):
            compute_kinematik([2, 0, stand_y, down_pos + 0.18])#change direction
            compute_kinematik([2, 0, stand_y_original-0.02, down_pos+0.05])#down
            open_gripper()
            detach("grasper", "wrist_3_link", "brick"+str(attached_brick), "link")
            compute_kinematik([2, 0, stand_y, down_pos + 0.18])#up

        else:
            if(model_side == 1):
                rotate_wrist(np.pi, 1)
            else:
                if(model_direction == 0):
                    rotate_wrist(-np.pi/2, 1)
                else:
                    rotate_wrist(np.pi/2, 1)

            compute_kinematik([0, 0.02, -0.405, down_pos, 2], True)#down
            open_gripper()
            detach("grasper", "wrist_3_link", "brick"+str(attached_brick), "link")

            compute_kinematik([0, 0, -0.3, down_pos, 2])#back
            compute_kinematik([0, 0, -0.3, down_pos + 0.18, 2])#up

            compute_kinematik([2, 0, -0.3, down_pos + 0.18])#change rotation

            compute_kinematik([2, 0, stand_y_original, down_pos + 0.18])#forward
        redetect = True


    if(redetect):
        time.sleep(1)

        image = bridge.imgmsg_to_cv2(last_image)
        angle, pose1, box, center = rec.getPose(image)
        xdiff = pose1[0]
        ydiff = pose1[1]
        xdiff = xdiff * 0.143 / 418
        ydiff = ydiff * 0.046 / 134
        thetas = compute_kinematik([2, posx + xdiff, posy - ydiff, down_pos + 0.18], True)
        angle = np.abs(angle)
        angle = np.deg2rad(angle)
        angle = round(angle, 2)
        rotate_wrist(angle, 1)
        time.sleep(1)

        last_hbrick = 0.32 - hdist
        pre_image = bridge.imgmsg_to_cv2(last_image)
        rec_thread = threading.Thread(target=detect, args=("dataset.pt", True,), kwargs={})
        rec_thread.start()


        compute_kinematik([2, posx, posy, down_pos], True)
        attach("grasper", "wrist_3_link", "brick"+str(attached_brick), "link")
        close_gripper()
        compute_kinematik([2, posx, posy, down_pos + 0.18])


        rec_thread.join()

        if(model_side != 0):
            adjust()

def detect_brick_position():
    global rec_thread, last_hbrick, pre_image, hside, brick_pixels_height, brick_pixels_box, model_direction, model_direction2

    open_gripper()

    mode = 2

    index_min = np.argmin(depth_ranges)
    angle = (index_min * 180 / nlasers)
    rad_angle = np.deg2rad(angle)
    ydist = math.cos(rad_angle) * depth_ranges[index_min]
    xdist = math.sin(rad_angle) * depth_ranges[index_min]

    jump = 0.05

    left = index_min - 1
    dr = depth_ranges
    while(left >= 0):
        if(np.abs(dr[left + 1] - dr[left]) > jump):
            break
        left -= 1

    right = index_min + 1
    dr = depth_ranges
    while(right < len(dr)):
        if(np.abs(dr[right - 1] - dr[right]) > jump):
            break
        right += 1

    left += 1
    right -= 1

    angle = (left * 180 / nlasers)
    rad_angle = np.deg2rad(angle)
    yleft = math.cos(rad_angle) * depth_ranges[left]
    xleft = math.sin(rad_angle) * depth_ranges[left]

    angle = (right * 180 / nlasers)
    rad_angle = np.deg2rad(angle)
    yright = math.cos(rad_angle) * depth_ranges[right]
    xright = math.sin(rad_angle) * depth_ranges[right]

    finalx = ((xleft + xright - xdist) - xdist)/2 + xdist
    finaly = ((yleft + yright - ydist) - ydist)/2 + ydist

    thetas = compute_kinematik([mode, finaly, finalx, -0.2])

    time.sleep(1)

    image = bridge.imgmsg_to_cv2(last_image)
    angle, pose1, box, center = rec.getPose(image)

    xdiff = pose1[0]
    ydiff = pose1[1]
    xdiff = xdiff * 0.143 / 418
    ydiff = ydiff * 0.046 / 134
    thetas = compute_kinematik([mode, posx - xdiff, posy + ydiff, -0.2], True)

    angle = np.abs(angle)
    angle = np.deg2rad(angle)
    angle = round(angle, 2)
    rotate_wrist(angle, 1)
    

    time.sleep(1)
    hside = 0.32 - hdist

    last_hbrick = 0.32 - hdist
    pre_image = bridge.imgmsg_to_cv2(last_image)
    
    rec_thread = threading.Thread(target=detect, args=("dataset.pt",), kwargs={})
    rec_thread.start()

    _, _, box, center = rec.getPose(pre_image)
    model_direction  = 0
    model_direction2 = 0
    if(center[1] < int((box[3] + box[2])/2)):
        model_direction = 1
    if(center[0] < int((box[1] + box[0])/2)):
        model_direction2 = 1
    brick_pixels_height = box[3] - box[2]
    brick_pixels_box = box

    

def application():
    global attached_model, brick_rot, rec_thread, first_bricks, total_bricks, KPI_12, KPI_21

    print("Press enter to start...")
    input()

    compute_kinematik([2, 0, 0.5, -0.1], False, True, 7)

    for i in range(total_bricks):
        compute_kinematik([2, 0, 0.7, 0], False, True, 2, True)


        if(i == 0):
            KPI_21 = datetime.now()

        KPI_11 = datetime.now()
        detect_brick_position()
        KPI_11 = datetime.now() - KPI_11

        if(sys.argv[1] == "a1"):
            print("KPI 1-1: " + str(KPI_11))

        thetas = compute_kinematik([2, posx, posy, -0.2], True, True, 2, True)
        thetas = compute_kinematik([2, posx, posy, -0.3], True, True, 4)
        thetas = compute_kinematik([2, posx, posy], True, True, 4)
        attach_joints(get_model_id())
        close_gripper()

        thetas = compute_kinematik([2, posx, posy, -0.3], True, True, 4)
        thetas = compute_kinematik([2, posx, posy, -0.2], True, True, 2)

        compute_kinematik([2, 0, -0.5, 0], False, False)

        rec_thread.join()

        adjust()


        x, y = spa.get_xy_ground_pos(attached_model)
        z = -0.38 + (nblocks[attached_model] * BLOCKS_HEIGHT[attached_model])
        nblocks[attached_model] += 1
        compute_kinematik([2, x, y, z+0.4], False, True, 10, True)

        rotate_wrist(brick_rot, 1, True)

        compute_kinematik([2, x, y, z+0.15], True, True, 10)
        compute_kinematik([2, x, y, z], True, True, 10)

        if(sys.argv[1] == "a1"):
            KPI_12 = datetime.now() - KPI_12
            print("KPI 1-2: " + str(KPI_12))
        elif(i == total_bricks-1 and sys.argv[1] == "a2"):
            KPI_21 = datetime.now() - KPI_21
            print("KPI 2-1: " + str(KPI_21))
        elif(i == total_bricks-1 and sys.argv[1] == "a3"):
            KPI_21 = datetime.now() - KPI_21
            print("KPI 3-1: " + str(KPI_21))

        time.sleep(0.3)

        open_gripper()

        if(first_bricks[attached_model] > 0):
            attach_bricks(attached_model)
        else:
            attach("brick"+str(attached_brick), "link", "brick_ground"+str(attached_model+1), "link")
            first_bricks[attached_model] = int(attached_brick)

        detach_joints()
        time.sleep(0.3)

        compute_kinematik([2, x, y, z+0.2], True, True, 10)


def compute_kinematik(args, ignorew3 = False, wait = True, max_wait = 8, lift_first = False):
    mode = int(args[0])
    x = float(args[1])
    y = float(args[2])
    zposition = -0.38
    if(len(args) > 3):
        zposition = float(args[3])
    matrix_mode = 1
    if(len(args) > 4):
        matrix_mode = int(args[4])

    global posx, posy
    posx = x
    posy = y 

    matrix = mat([
        [1, 0, 0, -x],
        [0, -1, 0, -y],
        [0, 0, -1, zposition],
        [0, 0, 0, 1]
             ])

    if(matrix_mode == 2):
        matrix = mat([ #mode 2 on y<=0 from down
            [1, 0, 0, -x],
            [0, 0, 1, -y],
            [0, 1, 0, zposition],
            [0, 0, 0, 1]
                 ])
    elif(matrix_mode == 3):
        matrix = mat([ #mode 0 on y<=0 from right
            [0, 0, -1, -x],
            [0, 1, 0, -y],
            [1, 0, 0, zposition],
            [0, 0, 0, 1]
                 ])
    elif(matrix_mode == 4):
        matrix = mat([ #mode 0 on y<=0 from left
            [0, 0, 1, -x],
            [0, 1, 0, -y],
            [1, 0, 0, zposition],
            [0, 0, 0, 1]
                 ])

    thetas = kin.invKine((matrix))

    if(lift_first):
        move(SHOULDER_LIFT, thetas[1,mode])
        move(ELBOW, thetas[2,mode])
        until_in_range({SHOULDER_LIFT : thetas[SHOULDER_LIFT, mode], ELBOW : thetas[ELBOW, mode]}, 3)

    w1rot = rotate(WRIST1, thetas[3,mode], False)
    w2rot = rotate(WRIST2, thetas[4,mode], False)

    w3rot = thetas[SHOULDER_PAN,mode] + np.pi/2
    if(y < 0):
        w3rot -= np.pi
    if(not ignorew3):
        w3rot = rotate_wrist(w3rot, 0, False)

    if(not lift_first):
        move(SHOULDER_LIFT, thetas[1,mode])
        move(ELBOW, thetas[2,mode])

    pan_rot = rotate(SHOULDER_PAN, thetas[0,mode], False)

    if(wait and ignorew3):
        if(lift_first):
            until_in_range({
                SHOULDER_PAN  : pan_rot,
                WRIST1        : w1rot,
                WRIST2        : w2rot,
            }, max_wait)
        else:
            until_in_range({
                SHOULDER_PAN  : pan_rot,
                SHOULDER_LIFT : thetas[SHOULDER_LIFT, mode],
                ELBOW         : thetas[ELBOW, mode],
                WRIST1        : w1rot,
                WRIST2        : w2rot,
            }, max_wait)
    elif(wait):
        if(lift_first):
            until_in_range({
                SHOULDER_PAN  : pan_rot,
                WRIST1        : w1rot,
                WRIST2        : w2rot,
                WRIST3        : w3rot,
            }, max_wait)
        else:
            until_in_range({
                SHOULDER_PAN  : pan_rot,
                SHOULDER_LIFT : thetas[SHOULDER_LIFT, mode],
                ELBOW         : thetas[ELBOW, mode],
                WRIST1        : w1rot,
                WRIST2        : w2rot,
                WRIST3        : w3rot,
            }, max_wait)

    return thetas


def main():

    global spawn_model, delete_model, mstate_srv, total_bricks
    if(len(sys.argv) > 1):
        init()
        if(sys.argv[1] == "a1"):
            total_bricks = 1
            spa.spawn(sys.argv, spawn_model, delete_model, mstate_srv)
            application()

        elif(sys.argv[1] == "a2"):
            total_bricks = len(BLOCKS)
            spa.spawn(sys.argv, spawn_model, delete_model, mstate_srv)
            application()

        elif(sys.argv[1] == "a3"):
            num = 11
            if(len(sys.argv) > 2):
                num = int(sys.argv[2])
            total_bricks = num
            
            spa.spawn(sys.argv, spawn_model, delete_model, mstate_srv)
            application()

        else:
            print("Invalid command")
        
    else:
        print("Invalid command")



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
