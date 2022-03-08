#!/usr/bin/env python

# Python
import numpy as np
from time import time
import rospkg

# Crazyswarm
from pycrazyswarm import *

# ROS
import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *
from crazyswarm_lnkd.srv import *

# Crazyswarm
from crazyswarm.msg import GenericLogData


def initSwarm():
    print("CF swarm starting...")
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper

    return swarm, timeHelper


if __name__ == "__main__":
    # Setup
    swarm, timeHelper = initSwarm()
    cf = swarm.allcfs.crazyflies[0]
    MR_checker = rospy.ServiceProxy(
        "MR_checker", MRProximityAlert, persistent=True)

    # Takeoff
    cf.takeoff(targetHeight=0.4, duration=2)
    timeHelper.sleep(2.5)

    # Navigate the forest for 3m
    totalXMove = 0
    while totalXMove < 3.0:

        # Land after movements
    input("\nPress any key to land...")
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
