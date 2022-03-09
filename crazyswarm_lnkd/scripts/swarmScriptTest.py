#!/usr/bin/env python

# Python
import numpy as np
from time import time
import rospkg

# Crazyswarm
from pycrazyswarm import *


def initSwarm():
    print("CF swarm starting...")

    # Build swarm objects from .yaml file
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper

    # Set params for UWB positioning mode (TDoA3) and robust estimation method
    swarm.allcfs.setParam("loco/mode", 3)
    swarm.allcfs.setParam("kalman/robustTdoa", 1)

    return swarm, timeHelper


def fblrMovementDemo(swarm: Crazyswarm):
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=1.0, duration=2)
    timeHelper.sleep(2.5)
    allcfs.goTo([0, 1.0, 0], 0, 2.0)    # left
    timeHelper.sleep(3.0)
    allcfs.goTo([-1.0, 0, 0], 0, 2.0)   # back
    timeHelper.sleep(3.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(3.0)
    allcfs.goTo([0, -1.0, 0], 0, 2.0)   # right
    timeHelper.sleep(3.0)
    allcfs.goTo([0, 0, 0.3], 0, 2.0)    # up
    timeHelper.sleep(3.0)


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

    # Following code is here as an example only, do NOT uncomment
    # Wherever the CFs take off from, move them into their correct default positions
    """
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(2.5)
    """


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

    fblrMovementDemo(swarm)

    input("\nPress any key to land...")
    swarm.allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
