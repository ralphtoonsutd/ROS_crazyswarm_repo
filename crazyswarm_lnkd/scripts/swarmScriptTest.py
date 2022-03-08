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


def circleArrangement(swarm: Crazyswarm, centreCoord):
    # Takes centre co-ordinates and moves the swarm into a circle around them

    for num, cf in enumerate(swarm.allcfs.crazyflies):
        # 1. Create drone co-ord from its formation position and the formation's centre
        formationPos = [0.0, 0.0, 0.0]
        for count, initialPos in enumerate(cf.initialPosition):
            formationPos[count] = centreCoord[count] + initialPos[count]

        # 2. Split drones into 5 different movement planes
        height = calcMovePlaneValue(num, 5)
        cf.goTo([0, 0, height], 0, 2.5, relative=True)

        # 3. Move to position and return to previous height
        cf.goTo(formationPos, 0, 2.5, relative=False)
        cf.goTo([0, 0, -height], 0, 2.5, relative=True)


def lineArrangement(swarm: Crazyswarm, startCoord):
    # Takes co-ord for start of the line and moves CFs into formation behind
    for num, cf in enumerate(swarm.allcfs.crazyflies):
        # 1. Create drone co-ord in the line
        formationPos = startCoord + [num*-0.25, 0, 0]

        # 2. Split drones into 5 different movement planes
        height = calcMovePlaneValue(num, 5)
        cf.goTo([0, 0, height], 0, 2.5, relative=True)

        # 3. Move to position and return to previous height
        cf.goTo(formationPos, 0, 2.5, relative=False)
        cf.goTo([0, 0, -height], 0, 2.5, relative=True)


def calcMovePlaneValue(droneNum, numPlanes):
    # This might break for number of planes different from 5 idfk
    height = (0.25*(numPlanes % droneNum)) - 0.5
    return height


if __name__ == "__main__":

    swarm, timeHelper = initSwarm()
    allcfs = swarm.allcfs

<<<<<<< Updated upstream
    allcfs.takeoff(targetHeight=0.4, duration=2)
    timeHelper.sleep(4)
=======
    allcfs.takeoff(targetHeight=1.0, duration=2)
    timeHelper.sleep(2.5)
>>>>>>> Stashed changes
    allcfs.goTo([0, 0.4, 0], 0, 1.0)  # left
    timeHelper.sleep(1.5)
    allcfs.goTo([0.4, 0, 0], 0, 1.0)  # forward
    timeHelper.sleep(1.5)
    allcfs.goTo([-0.4, 0, 0], 0, 1.0)  # back
    timeHelper.sleep(1.5)
    allcfs.goTo([0, -0.4, 0], 0, 1.0)  # right
    timeHelper.sleep(1.5)
    allcfs.goTo([0, 0, 0.2], 0, 1.0)
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
                timeHelper.sleep(0.5)
    except KeyboardInterrupt:
        allcfs.emergency()
    """
    input("\nPress any key to land...")

    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
