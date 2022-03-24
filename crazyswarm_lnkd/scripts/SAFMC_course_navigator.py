#!/usr/bin/env python

# SAFMC_course_navigator.py


##################################################################################################################################

# Script to complete the 2022 SAFMC CAT E competition course. Program basis is loading the trajectory coordinates and
# current formation from a .csv file, then writing the coordinates to the swarm. Coordinates saved in the .csv are
# given as the forward-most drone in the swarm, e.g. the top of the circle formation or the head of the conga-line
# formation.

# Program flow:
# 1. Load the .csv file and parse into a workable dictionary
# 2. Initialise the crazyswarm, takeoff and adjust the CFs into proper formation
# 3. Iterate through the co-ordinate dictionary moving the swarm as follows:
#   3A. If the next dictionary value is in a different formation, first perform the formation change and THEN execute
#       either 3B or 3C.
#   3B. If the drones are in the circle formation, perform a swarm relative move equal to the current global coord minus
#       the previous global coord (in the circle state the drones can always move together). For the first move, "rebuild"
#       the formation at the first global coord.
#   3C. If the drones are in the line formation, create a fresh coord array with length equal to num CFs to store their
#       positions. Initial values here should be the current values in the circle formation (absolute position), with
#       coords[0] = new position and coords[n] removed. Iterate over this array, performing global moves on a drone by
#       drone basis. For the following moves add the next coord from the trajectory dictionary to the top of the array,
#       remove the final value from the array, and write the values to the drones in order. The initial moves will cause
#       the circle formation to "unwrap" into the line formation.
# 4. When the end of the dictionary is reached, the drones should be in the circle formation above the landing zone. Perform
#    a final swarm adjustment to ensure a clean circle, then call the swarm to land.
# 5. Win and lift the trophy reminding your rivals it was EZPZ.

##################################################################################################################################


# Python
from cv2 import line
import numpy as np
import rospkg
import csv

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


# GLOBAL CONSTANTS
FORMATION_RADIUS = 1.25  # m


def initSwarm():
    # Load the crazyflies from file and build the swarm object
    print("CF swarm starting...")
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper
    timeHelper.sleep(3)

    # Reset the estimator to reduce RVIZ jank
    resetEstimator(swarm, timeHelper)

    return swarm, timeHelper


def stableTakeoff(swarm: Crazyswarm, timeHelper):
    # Performs a more stable takeoff facilitated by estimator reset
    resetEstimator(swarm, timeHelper)

    swarm.allcfs.takeoff(targetHeight=1.0, duration=4)
    timeHelper.sleep(4.0)


def circleLand(swarm: Crazyswarm, timeHelper, landCoord):
    circleArrangement(swarm, timeHelper, landCoord, avoidance=False)
    swarm.allcfs.land(0.04, 2.0)
    timeHelper.sleep(2.0)


def resetEstimator(swarm: Crazyswarm, timeHelper):
    swarm.allcfs.setParam("kalman/resetEstimation", 1)
    timeHelper.sleep(0.1)
    swarm.allcfs.setParam("kalman/resetEstimation", 0)
    timeHelper.sleep(1.0)


def circleArrangement(swarm: Crazyswarm, timeHelper, frontCoord, avoidance=True):
    # Takes front co-ordinates and arranges drones into circle formation
    # Assumes initial positions consider (0,0) to be front drone

    if avoidance:
        # 1. Split drones into 5 different movement planes
        for num, cf in enumerate(swarm.allcfs.crazyflies):
            height = calcFlightLevelValue(num, 5)
            cf.goTo([0, 0, height], 0, 2.5, relative=True)
        timeHelper.sleep(2.5)

    # 2. Create drone co-ord from its formation position and the formation's front and move
    for cf in swarm.allcfs.crazyflies:
        formationPos = np.add(cf.initialPosition, np.array(frontCoord))
        cf.goTo(formationPos, 0, 2.5, relative=False)
    timeHelper.sleep(2.5)

    if avoidance:
        # 3. Return to normal height
        for num, cf in enumerate(swarm.allcfs.crazyflies):
            height = calcFlightLevelValue(num, 5)
            cf.goTo([0, 0, -height], 0, 2.5, relative=True)
        timeHelper.sleep(2.5)


# Line arrangement shouldn't be needed (See 3C)
# def lineArrangement(swarm: Crazyswarm, timeHelper, startCoord):
#     # Takes co-ord for start of the line and arranges drones into line formation
#     # May need to be updated to handle longer lines/teams better

#     # 1. Split drones into 5 different movement planes
#     for num, cf in enumerate(swarm.allcfs.crazyflies):
#         height = calcFlightLevelValue(num, 5)
#         cf.goTo([0, 0, height], 0, 2.5, relative=True)
#     timeHelper.sleep(2.5)

#     # 2. Create drone co-ord from its formation number and the formation's front and move
#     for num, cf in enumerate(swarm.allcfs.crazyflies):
#         formationPos = np.add(np.array(startCoord),
#                               np.array([num*-0.25, 0, 0]))
#         cf.goTo(formationPos, 0, 2.5, relative=False)
#     timeHelper.sleep(2.5)

#     # 3. Return to normal height
#     for num, cf in enumerate(swarm.allcfs.crazyflies):
#         height = calcFlightLevelValue(num, 5)
#         cf.goTo([0, 0, -height], 0, 2.5, relative=True)
#     timeHelper.sleep(2.5)


def calcFlightLevelValue(droneNum, numLevels):
    # Returns absolute height for drone flight level
    # This might break for number of flight levels different from 5 idfk
    height = (0.25*(numLevels % droneNum)) - 0.5
    return height


def getCircleCoords(swarm: Crazyswarm, circlePosition):
    lineCoords = [[0, 0, 0]]*len(swarm.allcfs.crazyflies)

    for num, cf in enumerate(swarm.allcfs.crazyflies):
        lineCoords[num] = np.add(cf.initialPosition, np.array(circlePosition))

    return lineCoords


if __name__ == "__main__":
    # Setup swarm object
    swarm, timeHelper = initSwarm()

    # Multi-ranger code has been removed for now, potential to re-add in future
    # MR_checker = rospy.ServiceProxy("MR_checker", MRProximityAlert, persistent=True)

    previousCoord = [0, 0, 0]   # For circle moves
    currentFormation = 0        # 0 = circle, 1 = line
    lineCoords = [[0, 0, 0]]*len(swarm.allcfs.crazyflies)

    # Main program loop
    with open('testCSV.csv', newline='') as csvfile:
        # Load CSV coordinates into iterable object
        csv_Reader = csv.DictReader(csvfile, delimiter=' ', quotechar='|')

        # Takeoff
        stableTakeoff(swarm, timeHelper)
        # timeHelper.sleep(4)    UNCOMMENT IF NEEDED FOR STABILITY

        # 3. Iterate over coordate array and move the swarm
        for row in csv_Reader:
            currentCoord = np.array([row['x'], row['y'], row['z']])
            formation = row['Formation']

            # 3A - Formation update (FUNCTIONALLY COMPLETE)
            if formation != currentFormation:
                if formation == 0:
                    circleArrangement(swarm, timeHelper, currentCoord)
                else:
                    # I promise this makes sense :)
                    lineCoords = getCircleCoords(swarm, currentCoord)

                currentFormation = row['Formation']

            # 3B - Circle (FUNCTIONALLY COMPLETE)
            if currentFormation == 0:
                if row['ID'] != 1:
                    relativeMove = np.subtract(currentCoord, previousCoord)
                    swarm.allcfs.goTo(relativeMove, 0, 2.5, relative=True)
                else:
                    circleArrangement(swarm, timeHelper, currentCoord, False)

            # 3C - Line
            else:
                lineCoords.pop(len(lineCoords)-1)  # Remove final element
                lineCoords.insert(0, currentCoord)  # Add new coord to front

                # Write to drones
                for num, cf in enumerate(swarm.allcfs.crazyflies):
                    cf.goTo(lineCoords[num], 0, 2.5, relative=False)
                timeHelper.sleep(2.5)

            # Store the previous coord as DictReader object can't be indexed
            previousCoord = currentCoord

    # Land
    circleLand(swarm, timeHelper, previousCoord)

    # GG
