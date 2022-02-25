#! /usr/bin/python3

# PROGRAM STRUCTURE
# 1. Setup ROS node
# 2. Create listener for each crazyflie with MR deck (this will be listed in a separate .yaml file in crazyswarm_lnkd/launch)
# 3. Create service for returning data from callback to check proximity

# Current state: Only checks for ID CF2, this should be expanded to store all CFs with MR decks in an object


import rospy
import rospkg
from rosnode import kill_nodes

from crazyswarm.msg import GenericLogData
from pycrazyswarm import *
from crazyswarm_lnkd.srv import MRProximityAlert, MRProximityAlertResponse

PROXIMITYLIMIT = 400.0


class MRDrones:
    def __init__(self) -> None:
        # Store the MR values in a dictionary with format ID:[0,0,0,0]
        self.droneMRValues = {}
        self.setDroneIDs()

    def setDroneIDs(self):
        rospack = rospkg.RosPack()
        yamlPath = rospack.get_path(
            'crazyswarm_lnkd')+"/launch/crazyflies.yaml"
        swarm = Crazyswarm(crazyflies_yaml=yamlPath)

        self.droneMRValues.clear()
        for cf in swarm.allcfs.crazyflies:
            if cf.getParam("deck/bcMultiranger"):
                self.droneMRValues[cf.id] = [0, 0, 0, 0]

        rospy.loginfo(self.droneMRValues)

        kill_nodes("CrazyflieAPI")

    def updateMRValue(self, droneID, values):
        self.droneMRValues[droneID] = values

    def getMRValue(self, id):
        return self.droneMRValues.get(id)

    def getDroneIDs(self):
        return self.droneMRValues.keys()


def mrDataCallback(data, key):
    # Multiranger values are broadcast in following format:
    # ["range.front", "range.left", "range.back", "range.right"]
    mrDroneGroup.updateMRValue(key, data.values)


def sendMRStatus(req):
    if req.cfID in mrDroneGroup.getDroneIDs():
        data = mrDroneGroup.getMRValue(req.cfID)
        return MRProximityAlertResponse(data)


mrDroneGroup = MRDrones()

if __name__ == "__main__":
    responseSrv = rospy.Service('MR_checker', MRProximityAlert, sendMRStatus)

    for key in mrDroneGroup.getDroneIDs():
        topicName = '/cf' + str(key) + '/MR_values'
        rospy.Subscriber(topicName, GenericLogData,
                         callback=mrDataCallback, callback_args=key, queue_size=10)

    rospy.spin()
