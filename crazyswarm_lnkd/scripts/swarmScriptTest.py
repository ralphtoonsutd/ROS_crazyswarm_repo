#!/usr/bin/env python

# Python
import csv
from time import time
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

    
    allcfs.crazyflies[0].goTo([0, 0, 0.6], 0, 2)
    timeHelper.sleep(2.5)

    allcfs.crazyflies[0].goTo([0.5, 0.5, 0.6], 0, 2)
    allcfs.crazyflies[1].goTo([0, 0, 0.6], 0, 2)
    timeHelper.sleep(2.5)

    allcfs.goTo([-0.5, -0.5, 0.3], 0, 2)
    timeHelper.sleep(2.5)
    

    input("\nPress any key to land...")

    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
