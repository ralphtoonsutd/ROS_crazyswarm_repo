#!/usr/bin/env python

# Python
import csv
import rospy
import rospkg

# ROS
from geometry_msgs.msg import *
from std_msgs.msg import String

# Crazyswarm
from pycrazyswarm import *
from pycrazyswarm.crazyflie import TimeHelper


def initSwarm():
    print("CF swarm starting...")
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper

    return swarm, timeHelper


if __name__ == "__main__":

    swarm, timeHelper = initSwarm()
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=0.4, duration=2)
    timeHelper.sleep(2.5)

    rospy.Subscriber('instructions', Pose, moveCallback, swarm)

    print("Swarm ready for commands")
    print("Note: Run cf_get_bs_geometry.py if CF flight area bounds are incorrectly configured")

    # Program loop

    input("\nPress any key to land...")

    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
