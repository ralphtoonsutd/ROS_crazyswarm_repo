#!/usr/bin/env python

# Use the multiranger to navigate over an obsticle, stop, then land
# Multiranger values are broadcast in following format:
# ["range.front", "range.left", "range.back", "range.right"]

# Python
import time
from turtle import st
import numpy as np

# ROS
import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *
from crazyswarm_lnkd.srv import *

# Crazyswarm
from pycrazyswarm import *
from crazyswarm.msg import GenericLogData


def initSwarm():
    # rospy.wait_for_service("MR_checker")
    print("CF swarm starting...")
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper

    return swarm, timeHelper


def proximityCheck(data):
    print("checking distance")
    print("\rFront: %f | Left: %f | Back: %f | Right: %f"
          % (data.values[0], data.values[1], data.values[2], data.values[3]), end="\r")

    if data.values[0] < 200:
        tooClose = True

    elif data.values[0] > 300:
        tooClose = False


if __name__ == '__main__':
    # SETUP
    swarm, timeHelper = initSwarm()
    cf = swarm.allcfs.crazyflies[0]  # Only test with 1 cf this time

    MR_checker = rospy.ServiceProxy(
        "MR_checker", MRProximityAlert, persistent=True)
    status = MR_checker(1)

    print("Front: %f" % (status.MRAlert[0]), end="\r")

    # DO MOVES
    cf.takeoff(0.4, 1.0)
    timeHelper.sleep(1.5)

    # Move forward to wall
    while (status.MRAlert[0] == 0):
        cf.goTo([0.05, 0.0, 0.0], 0, 0.3, relative=True)
        timeHelper.sleep(0.4)
        status = MR_checker(1)
        print(status)
        timeHelper.sleep(1)

    """

    # Move up over wall height
    while(status.MRAlert[0] != 0):
        cf.goTo([0.0, 0.0, 0.3], 0, 0.5, relative=True)
        timeHelper.sleep(0.5)
        status = MR_checker(1)

    MR_checker.close()
    timeHelper.sleep(1.0)
    cf.goTo([1.0, 0.0, 0.0], 0, 1.5, relative=True)
    timeHelper.sleep(2.0)
    """
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
    cf.stop()
