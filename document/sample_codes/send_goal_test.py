#!/usr/bin/env python

# File: send_goal.py
# Abstract: Send goal to move_base through specific topic
# Author: HaoChih, LIN
# License: BSD (3-clause version)
# Email: hypha.ros@gmil.com
# Date: 2017/06/18

## Import libraries
import string
import rospy
import sys
from geometry_msgs.msg import PoseStamped

## ROS Python Init
rospy.init_node("send_goal")

## Parameters definition
frame_id = rospy.get_param('~frame_id', '/map')

rate = rospy.Rate(10) # 10hz
goalMsg = PoseStamped()
goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

## Show Start Info
rospy.loginfo("===== send_goal pyhton node =====")
rospy.loginfo("----- Param Info -----")
rospy.loginfo("frame_id: %s", frame_id)

## Start pub goal data to move_base
rospy.sleep(1.0)
if rospy.is_shutdown() == False:
    rospy.loginfo("Sending goal ...")
    goalMsg.header.stamp = rospy.Time.now()
    goalMsg.header.frame_id = frame_id
    goalMsg.header.seq = 0
    goalMsg.pose.position.x = 17.6;
    goalMsg.pose.position.y = 22;
    goalMsg.pose.orientation.z = 0.0;
    goalMsg.pose.orientation.w = 1.0;
    goalPub.publish(goalMsg)

