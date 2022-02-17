#!/usr/bin/env python
from pycrazyswarm import crazyswarm_py
from pycrazyswarm.crazyswarm_py import Crazyswarm
import rospy
import numpy as np
from std_msgs.msg import *  # Bit wasteful lmao
from geometry_msgs.msg import *
from crazyswarm.msg import GenericLogData


def callback(data):
    rospy.loginfo("%f", data[0]]
    rospy.loginfo("%f", data[1])
    #rospy.loginfo("%f", data.range.left)
    #rospy.loginfo("%f", data.range.right)


def batteryCallBack(data, cfNum):
    rospy.loginfo("%f: I heard %f", cfNum, data.values[0])


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("/cf231/pose/", PoseStamped, callback, queue_size=10)

    rospy.Subscriber('/cf2/MR_values', GenericLogData, callback, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
