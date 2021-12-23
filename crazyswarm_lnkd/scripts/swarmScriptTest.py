#!/usr/bin/env python

# Python
import numpy as np
from time import time
import rospkg

# Crazyswarm
from pycrazyswarm import *

circleIsh = [[0.0, 0.5, 0.0], [0.35, 0.357, 0.0], [0.5, 0.0, 0.0], [0.35, -0.357, 0.0],
             [0.0, -0.5, 0.0], [-0.35, -0.357, 0.0], [-0.5, 0.0, 0.0], [-0.35, 0.357, 0.0]]


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

    # Wherever the CFs take off from, move them into their correct default positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    # Do the circle movement and break on ctrl+C
    try:
        while True:
            for point in circleIsh:
                for cf in allcfs.crazyflies:
                    pos = np.array(cf.initialPosition) + np.array(point)
                    cf.goTo(pos, 0, 2.0)
                timeHelper.sleep(1.8)
    except KeyboardInterrupt:
        pass

    input("\nPress any key to land...")

    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
