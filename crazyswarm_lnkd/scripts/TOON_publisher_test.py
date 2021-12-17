#!/usr/bin/env python

import rospy

# A bit wasteful - work on more specific imports when datatypes are known better
from std_msgs.msg import *
from geometry_msgs.msg import *


def talker():
    """
    pub = rospy.Publisher('instructions', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
    """
    pub = rospy.Publisher('instructions', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # User enters 3 values to control drone position
        x = float(input("x: "))
        y = float(input("y: "))
        z = float(input("z: "))

        thingtosend = Pose()
        thingtosend.position.x, thingtosend.position.y, thingtosend.position.z = x, y, z
        pub.publish(thingtosend)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
