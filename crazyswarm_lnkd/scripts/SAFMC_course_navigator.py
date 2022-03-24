#!/usr/bin/env python

# SAFMC_course_navigator.py


##################################################################################################################################

# Script to complete the 2022 SAFMC CAT E competition course. Program basis is loading the trajectory coordinates and
# current formation from a .csv file, then writing the coordinates to the swarm. Coordinates saved in the .csv are
# given as the forward-most drone in the swarm, e.g. the front/top of the circle formation or the head of the conga-line
# formation.

# Program flow:
# 1. Load the .csv file and parse into a workable dictionary
# 2. Initialise the crazyswarm, takeoff and adjust the CFs into proper formation
# 3. Iterate through the co-ordinate dictionary moving the swarm as follows:
#   3A. If the drones are in the circle formation, perform a swarm relative move equal to the next global coord minus
#       the current global coord (in the circle state the drones can always move together)
#   3B. If the drones are in the line formation, create a fresh coord array with length equal to num CFs to store their
#       positions. Initial value here should be the current value in the .csv dictionary (for lead drone), with following
#       values being equal to (CURRENT_X, CURRENT_Y - (DRONE_SPACING*ARRAY_POSITION), CURRENT_Z). Iterate over this array,
#       performing global moves on a drone by drone basis. For the following moves add the next coord from the trajectory
#       dictionary to the top of the array, remove the final value from the array, and write the values to the drones in order.
#   3C. If the next dictionary value is in a different formation, first perform the formation change and THEN execute
#       either 3A or 3B.
# 4. When the end of the dictionary is reached, the drones should be in the circle formation above the landing zone. Perform
#    a final swarm adjustment to ensure a clean circle, then call the swarm to land.
# 5. Win and lift the trophy reminding your rivals it was EZPZ and they're trash tier.

##################################################################################################################################


# IMPORTS #

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
