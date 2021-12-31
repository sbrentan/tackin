#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math

def talker():
    shoulder_pan_pub = rospy.Publisher('/test2/shoulder_pan_joint_position_controller/command', Float64, queue_size=10)
    shoulder_lift_pub = rospy.Publisher('/test2/shoulder_lift_joint_position_controller/command', Float64, queue_size=10)
    elbow_pub = rospy.Publisher('/test2/elbow_joint_position_controller/command', Float64, queue_size=10)
    wrist1_pub = rospy.Publisher('/test2/wrist_1_joint_position_controller/command', Float64, queue_size=10)
    wrist2_pub = rospy.Publisher('/test2/wrist_2_joint_position_controller/command', Float64, queue_size=10)
    wrist3_pub = rospy.Publisher('/test2/wrist_3_joint_position_controller/command', Float64, queue_size=10)
    finger1_pub = rospy.Publisher('/test2/finger_1_joint_position_controller/command', Float64, queue_size=10)
    finger2_pub = rospy.Publisher('/test2/finger_2_joint_position_controller/command', Float64, queue_size=10)
    finger3_pub = rospy.Publisher('/test2/finger_3_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('supreme_commander', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    shoulder_pan_pub.publish(1.57/2)
    shoulder_lift_pub.publish(-1.57/2)
    elbow_pub.publish(1.57/2)
    wrist1_pub.publish(-1.57)
    wrist2_pub.publish(-1.57)
    wrist3_pub.publish(0.5)


    #while not rospy.is_shutdown():
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass