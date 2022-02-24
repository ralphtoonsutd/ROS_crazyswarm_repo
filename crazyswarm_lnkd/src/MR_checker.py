#!/usr/bin/env python

# PROGRAM STRUCTURE
# 1. Setup ROS node
# 2. Create listener for each crazyflie with MR deck (this will be listed in a separate .yaml file in crazyswarm_lnkd/launch)
# 3. Create service for returning data from callback to check proximity

# Current state: Only checks for ID CF2, this should be expanded to store all CFs with MR decks in an object


import rospy
from crazyswarm.msg import GenericLogData
from crazyswarm_lnkd.srv import MRProximityAlert, MRProximityAlertResponse

# Globals (for now, later use class/dictionary to handle)
mrStatus = [0, 0, 0, 0]  # Default all sensors warnings to be false

PROXIMITYLIMIT = 200.0


def mrListener(data):
    # Multiranger values are broadcast in following format:
    # ["range.front", "range.left", "range.back", "range.right"]
    global mrStatus

    for i, value in enumerate(data.values):
        if value < PROXIMITYLIMIT:
            mrStatus[i] = 1
        else:
            mrStatus[i] = 0

    print("\rFront: %f | Left: %f | Back: %f | Right: %f"
          % (data.values[0], data.values[1], data.values[2], data.values[3]), end="\r")


def sendMRStatus(arg):
    return MRProximityAlertResponse(mrStatus)


if __name__ == "__main__":
    rospy.init_node('MR_checker')
    responseSrv = rospy.Service('MR_checker', MRProximityAlert, sendMRStatus)
    rospy.Subscriber('/cf2/MR_values', GenericLogData,
                     mrListener, queue_size=10)

    rospy.spin()
