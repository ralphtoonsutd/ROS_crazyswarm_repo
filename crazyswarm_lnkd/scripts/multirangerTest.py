#!/usr/bin/env python

# Use the multiranger to navigate over an obsticle, stop, then land
# Multiranger values are broadcast in following format:
# ["range.front", "range.left", "range.back", "range.right"]

# Python
from time import time
import numpy as np

# ROS
import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *

# Crazyswarm
from pycrazyswarm import *
from crazyswarm.msg import GenericLogData

# GLOBAL BLEGH
tooClose = False


def initSwarm():
    print("CF swarm starting...")
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper

    return swarm, timeHelper


def proximityCheck(data):
    print("\rFront: %f | Left: %f | Back: %f | Right: %f"
          % (data.values[0], data.values[1], data.values[2], data.values[3]), end="\r")

    if data.values[0] < 200:
        tooClose = True

    else:
        tooClose = False


if __name__ == '__main__':
    # SETUP
    swarm, timeHelper = initSwarm()
    cf = swarm.allcfs.crazyflies[0]  # Only test with 1 cf this time

    rospy.init_node('MR_Logger', anonymous=True)
    rospy.Subscriber('/cf2/MR_values', GenericLogData,
                     proximityCheck, queue_size=10)

    # DO MOVES
    cf.takeoff(0.4)
    timeHelper.sleep(1.0)

    # Move forward to wall
    while (not tooClose):
        cf.goto([0.1, 0.0, 0.0], 0, 0.3, relative=True)
        timeHelper.sleep(0.4)

    # Move up over wall height
    while(tooClose):
        cf.goto([0.0, 0.0, 0.3], 0, 0.5, relative=True)
        timeHelper.sleep(0.5)

    timeHelper.sleep(1.0)

    cf.goto([1.0, 0.0, 0.0], 0, 1.5, relative=True)
    timeHelper.sleep(2.0)

    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
    cf.stop()
