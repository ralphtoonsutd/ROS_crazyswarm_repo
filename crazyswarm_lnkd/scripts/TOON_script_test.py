#!/usr/bin/env python

import csv

import rospy
from geometry_msgs.msg import *
from pycrazyswarm import *
from pycrazyswarm.crazyflie import TimeHelper
from std_msgs.msg import String

"""
class SwarmHandler:
    def __init__(self) -> None:
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.limits = Point()
        self.lighthousePos = [Point(), Point()]

    def setLighthouseLimits(self):
        # Read the calculated limits from file
        with open('bs_geometery.csv', 'r', newline='') as csvfile:
            csvfile = csv.reader(csvfile, delimiter=' ', quotechar='|')
            for row in csvfile:
                self.lighthousePos[csvfile.line_num].x = row[0]
                self.lighthousePos[csvfile.line_num].y = row[1]
                self.lighthousePos[csvfile.line_num].z = row[2]

        # For each axes, calculate limits from
        if self.lighthousePos[0].z >= self.lighthousePos[1].z:
            self.limits.z = self.lighthousePos[0].z - 5
        else:
            self.limits.z = self.lighthousePos[1].z - 5

    def safeMove(self):
        # Take the move position and make sure its within the current safe move space

        # 1) Ensure co-ords are within the current move space
        # 1.1) If move is relative, take current position + move and compare with limits
        # 1.2) If move is absolute, take move and compare with limits directly
        # 2)
        pass
"""


def initSwarm():
    print("CF swarm starting...")
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    return swarm, timeHelper


'''
def moveCallback(data, swarm):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.position.x)
    swarm.allcfs.crazyflies[0].goTo(
        [data.position.x, data.position.y, data.position.z], 0, 2.0)
'''

if __name__ == "__main__":

    #swarm, timeHelper = initSwarm()

    swarm = Crazyswarm(crazyflies_yaml="crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.crazyflies[0].takeoff(targetHeight=0.4, duration=2)
    timeHelper.sleep(2.5)

    #allcfs.goTo([1, 0, 0.7], 2, 1.5)
    #timeHelper.sleep(1.5)

    #rospy.Subscriber('instructions', Pose, moveCallback, swarm)

    print("Swarm ready for commands")
    print("Note: Run cf_get_bs_geometry.py if CF flight area bounds are incorrectly configured")

    # Program loop

    input("\nPress any key to land...")

    allcfs.crazyflies[0].land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
