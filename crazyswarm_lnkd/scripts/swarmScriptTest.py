#!/usr/bin/env python

# Python
import numpy as np
from time import time
import rospkg

# Crazyswarm
from pycrazyswarm import *

circleIsh = [[0.0, 0.5, 0.6], [0.35, 0.357, 0.6], [0.5, 0.0, 0.6], [0.35, -0.357, 0.6],
             [0.0, -0.5, 0.6], [-0.35, -0.357, 0.6], [-0.5, 0.0, 0.6], [-0.35, 0.357, 0.6]]

verticalCircleIsh = [[0.5, 0.0, 0.6], [0.29, 0.0, 0.42], [0.0, 0.0, 0.38], [-0.29, 0.0, 0.42],
                     [-0.5, 0.0, 0.6], [-0.29, 0.0, 0.78], [0.0, 0.0, 0.82], [0.29, 0.0, 0.78]]


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
    allcfs.goTo([0, 0.4, 0], 0, 1.0)
    timeHelper.sleep(1.5)

    # Wherever the CFs take off from, move them into their correct default positions
    """
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(2.5)
    """
    # Do the circle movement and break on ctrl+C
    """
    try:
        while True:
            for count in range(0, 8, 1):
                allcfs.goTo(circleIsh[count], 0, 1)
                #allcfs.crazyflies[1].goTo(verticalCircleIsh[count], 0, 1)
                timeHelper.sleep(0.5)
    except KeyboardInterrupt:
        pass
    """
    input("\nPress any key to land...")

    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
