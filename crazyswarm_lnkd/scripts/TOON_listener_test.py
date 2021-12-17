#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import *  # Bit wasteful lmao
from geometry_msgs.msg import *


def callback(data):
    rospy.loginfo("%f", data.position.x)
    rospy.loginfo("%f", data.position.y)
    rospy.loginfo("%f", data.position.z)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("/cf231/pose/", PoseStamped, callback, queue_size=10)

    rospy.Subscriber('instructions', Pose, callback, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
