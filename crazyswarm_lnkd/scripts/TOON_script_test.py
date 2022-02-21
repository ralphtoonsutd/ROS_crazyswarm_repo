#!/usr/bin/env python

# Python
import csv

# ROS
import rospy
import rospkg
from geometry_msgs.msg import *
from std_msgs.msg import String

# Crazyswarm
from pycrazyswarm import *
#from pycrazyswarm.crazyflie import TimeHelper


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
    rospack = rospkg.RosPack()
    launchPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=launchPath)
    timeHelper = swarm.timeHelper

    return swarm, timeHelper


def moveCallback(data, swarm):
    # Test print to ensure correct co-ords are being sent
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.position.x)
    swarm.allcfs.goTo(
        [data.position.x, data.position.y, data.position.z], 0, 2.0)


def batteryCallBack(data, ):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data)


if __name__ == "__main__":

    swarm, timeHelper = initSwarm()
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=0.4, duration=2)
    timeHelper.sleep(2.5)

    rospy.Subscriber('instructions', Pose, moveCallback, swarm)

    print("Swarm ready for commands")
    print("Note: Run cf_get_bs_geometry.py if CF flight area bounds are incorrectly configured")

    # Program loop

    input("\nPress enter key to land...")

    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3)
