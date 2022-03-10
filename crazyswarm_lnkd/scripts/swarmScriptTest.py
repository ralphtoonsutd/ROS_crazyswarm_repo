#!/usr/bin/env python

# Python
import numpy as np
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


def fblrMovementDemo(swarm: Crazyswarm, timeHelper):
    allcfs = swarm.allcfs
    allcfs.goTo([0, 1.0, 0], 0, 2.0)    # left
    timeHelper.sleep(5.0)
    allcfs.goTo([-1.0, 0, 0], 0, 2.0)   # back
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([0, -1.0, 0], 0, 2.0)   # right
    timeHelper.sleep(5.0)
    allcfs.goTo([0, 0, 0.3], 0, 2.0)    # up
    timeHelper.sleep(5.0)


def wallScam(swarm: Crazyswarm, timeHelper):
    # THIS DOESNT ACTUALLY COMPLETE THE WALL :)
    # Just works through a predefined list of moves, with no sensor inputs

    # Move forward up to wall, then go up, over, then down
    # Check if z-ranger needs to be turned off while clearing the wall, it might make the drones jump
    allcfs = swarm.allcfs
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # 1m forwards
    timeHelper.sleep(2.5)

    allcfs.goTo([0, 0, 1.8], 0, 2.5)    # 1.8m up, gives total height of 2.8m
    timeHelper.sleep(3.0)
    allcfs.setParam("tof/enable", 0)
    timeHelper.sleep(2.0)
    # allcfs.goTo([0, 0, 0.5], 0, 2.5)    # 1.8m up, gives total height of 2.8m
    # timeHelper.sleep(3.0)

    allcfs.goTo([2.0, 0, 0], 0, 3.0)    # 2m forward should clear all drones
    timeHelper.sleep(4.0)
    # allcfs.goTo([0, 0, -1.5], 0, 2.5)   # Return to normal flight height
    # timeHelper.sleep(3.0)

    #allcfs.setParam("tof/enable", 1)
    timeHelper.sleep(2.0)
    allcfs.goTo([0, 0, -1.5], 0, 2.5)   # Return to normal flight height
    timeHelper.sleep(3.0)


def bambooForestScam(swarm: Crazyswarm, timeHelper):
    # THIS DOESNT ACTUALLY COMPLETE THE FOREST :)
    # Just works through a predefined list of co-ordinates, with no sensor inputs

    # MAKE SURE DRONES ARE SET UP IN A LINE FORMATION FOR THIS DEMO

    # These are ABSOLUTE co-ordinates
    forestPath = [[0, 2.0, 0.7], [0.40, 2.0, 0.7], [0.80, 2.0, 0.7], [
        1.2, 2.0, 0.7], [1.6, 2.0, 0.7], [2.1, 2.0, 0.7], [2.45, 2.6, 0.7], [3.0, 2.0, 0.7], [3.5, 2.0, 0.7], [3.9, 2.0, 0.7], [4.3, 2.0, 0.7], [4.7, 2.0, 0.7], [5.1, 2.0, 0.7]]

    for n in range(0, len(forestPath)-5):
        for m, cf in enumerate(swarm.allcfs.crazyflies):
            cf.goTo(forestPath[n+4-m], 0, 2.0)
        timeHelper.sleep(1.8)


def hoopScam(swarm: Crazyswarm, timeHelper):
    # THIS DOESNT ACTUALLY COMPLETE THE HOOPS :)
    # Just works through a predefined list of co-ordinates, with no sensor inputs
    allcfs = swarm.allcfs
    HH = 1.4    # Hoop height
    # MAKE SURE DRONES ARE SET UP IN A LINE FORMATION FOR THIS DEMO
    hoopPath = [[0, 2.0, HH], [0.4, 2.0, HH], [0.8, 2.0, HH], [
        1.2, 2.0, HH], [1.6, 2.0, HH], [2.1, 3.0, HH], [2.6, 3.0, HH], [3.1, 2.0, HH], [3.7, 2.0, HH], [4.1, 2.0, HH], [4.5, 2.0, HH], [4.9, 2.0, HH], [5.3, 2.0, HH]]

    allcfs.setParam("tof/enable", 0)
    timeHelper.sleep(2.0)
    for n in range(0, len(hoopPath)-5):
        for m, cf in enumerate(swarm.allcfs.crazyflies):
            cf.goTo(hoopPath[n+4-m], 0, 2.0)
        timeHelper.sleep(3.0)

    timeHelper.sleep(3.0)
    allcfs.setParam("tof/enable", 1)
    timeHelper.sleep(2.0)


def circleArrangement(swarm: Crazyswarm, timeHelper, centreCoord):
    # Takes centre co-ordinates and moves the swarm into a circle around them

    # 1. Split drones into 5 different movement planes
    for num, cf in enumerate(swarm.allcfs.crazyflies):
        height = calcMovePlaneValue(num, 5)
        cf.goTo([0, 0, height], 0, 2.5, relative=True)

    timeHelper.sleep(2.5)

    for num, cf in enumerate(swarm.allcfs.crazyflies):
        # 2. Create drone co-ord from its formation position and the formation's centre
        formationPos = [0.0, 0.0, 0.0]
        for count, initialPos in enumerate(cf.initialPosition):
            formationPos[count] = centreCoord[count] + initialPos[count]

        # 3. Move to position
        cf.goTo(formationPos, 0, 2.5, relative=False)

    timeHelper.sleep(2.5)

    # 4. Return to normal height
    for num, cf in enumerate(swarm.allcfs.crazyflies):
        height = calcMovePlaneValue(num, 5)
        cf.goTo([0, 0, -height], 0, 2.5, relative=True)

    timeHelper.sleep(2.5)

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

    # 1. Split drones into 5 different movement planes
    for num, cf in enumerate(swarm.allcfs.crazyflies):
        height = calcMovePlaneValue(num, 5)
        cf.goTo([0, 0, height], 0, 2.5, relative=True)

    timeHelper.sleep(2.5)

    for num, cf in enumerate(swarm.allcfs.crazyflies):
        # 2. Create drone co-ord from its formation position and the formation's centre
        formationPos = startCoord + [num*-0.25, 0, 0]
        # 3. Move to position
        cf.goTo(formationPos, 0, 2.5, relative=False)

    timeHelper.sleep(2.5)

    # 4. Return to normal height
    for num, cf in enumerate(swarm.allcfs.crazyflies):
        height = calcMovePlaneValue(num, 5)
        cf.goTo([0, 0, -height], 0, 2.5, relative=True)

    timeHelper.sleep(2.5)


def calcMovePlaneValue(droneNum, numPlanes):
    # This might break for number of planes different from 5 idfk
    height = (0.25*(numPlanes % droneNum)) - 0.5
    return height


if __name__ == "__main__":
    swarm, timeHelper = initSwarm()

    swarm.allcfs.setParam("kalman/resetEstimation", 1)
    timeHelper.sleep(0.1)
    swarm.allcfs.setParam("kalman/resetEstimation", 0)
    timeHelper.sleep(1.0)

    swarm.allcfs.takeoff(targetHeight=0.5, duration=2)
    timeHelper.sleep(5.0)
    
    allcfs = swarm.allcfs

    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)
    allcfs.goTo([1.0, 0, 0], 0, 2.0)    # forward
    timeHelper.sleep(5.0)

    # UNCOMMENT THE MOVE THAT YOU WANT TO DO FROM FOLLOWING LIST
    #fblrMovementDemo(swarm, timeHelper)
    #wallScam(swarm, timeHelper)
    #bambooForestScam(swarm, timeHelper)
    #hoopScam(swarm, timeHelper)
    # circleArrangement()

    input("\nPress any key to land...")
    swarm.allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
