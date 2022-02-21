#!/usr/bin/env python
from pycrazyswarm import crazyswarm_py
from pycrazyswarm.crazyswarm_py import Crazyswarm
import rospy
import numpy as np
from std_msgs.msg import *  # Bit wasteful lmao
from geometry_msgs.msg import *
from crazyswarm.msg import GenericLogData

# Multiranger values are broadcast in following format:
# ["range.front", "range.left", "range.back", "range.right"]


def callback(data):
    """
    rospy.loginfo("Front: %f", data.values[0])
    rospy.loginfo("Left: %f", data.values[1])
    rospy.loginfo("Back: %f", data.values[2])
    rospy.loginfo("Right: %f", data.values[3])
    """

    print("\rFront: %f | Left: %f | Back: %f | Right: %f"
          % (data.values[0], data.values[1], data.values[2], data.values[3]), end="\r")


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('MR_Logger', anonymous=True)

    #rospy.Subscriber("/cf231/pose/", PoseStamped, callback, queue_size=10)

    rospy.Subscriber('/cf2/MR_values', GenericLogData, callback, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
