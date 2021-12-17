#!/usr/bin/env python


# Following code is based on figure8_csv.py to attempt to make drone swarming work
# Simply takes the setup and middle section of the program to include takeoff and landing

import numpy as np

from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        print("Taking off")
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        print("Landing")
        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)
