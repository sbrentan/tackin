#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Duration
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ParallelGripperActionController:
   
    def __init__(self):
        rospy.init_node('test2_publisher')

        pub = rospy.Publisher('/test2/arm_controller/command', JointTrajectory, queue_size=10)

        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point=JointTrajectoryPoint()
        point.positions = [100, 100, 100, 100, 100, 100]
        point.time_from_start = rospy.Duration(2)
        joints_str.points.append(point)

        pub.publish(joints_str)
        rospy.loginfo("position updated")

if __name__=='__main__': 
    try:
        ParallelGripperActionController()
    except rospy.ROSInterruptException:
        pass