#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2020 - 2021 Bitcraze AB
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
#
#
#  Calculate Lighthouse base station geometry based on data from the
#  Crayzyflie. Requires a lighthouse deck.
#
#  This script connects to the Crazyflie and reads the sweep angles
#  for the base station(s) and calculates their position and orientation in
#  a coordinate system with origin at the position of the Crazyflie.
#
#  Usage:
#  1. Place the Crazyflie in the origin of your coordinate system, facing
#     positive X.
#  2. Make sure the Lighthouse deck is mounted completely parallell with the
#     ground (Crazyflie PCB) since this is what is going to define the
#     coordiate system.
#  3. Run the script
#  4. Copy/paste the output into lighthouse.c, recompile and flash the Crazyflie.
#
#  EDIT:
#  Script will now output base station positions ONLY to CSV in current directories
#  Script can handle multiple CFs and pulls URIs from crazyflies.yaml
#


import argparse
import logging
import csv
import numpy as np
from threading import Event
import rospkg
from pycrazyswarm import *
import yaml
from math import atan, cos, sin

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseBsGeoEstimator
from cflib.localization import LighthouseSweepAngleAverageReader


class Estimator:
    def __init__(self):
        self.sensor_vectors_all = None
        self.collection_event = Event()
        self.write_event = Event()

    def angles_collected_cb(self, angles):
        self.sensor_vectors_all = angles
        self.collection_event.set()

    def write_done_cb(self, success):
        if not success:
            print("Write to CF failed!")
        self.write_event.set()

    def estimate(self, ids, do_write, invert, invert2):
        cf = Crazyflie(rw_cache='./cache')
        geoStore = []
        geometries = {}
        print(ids)

        with SyncCrazyflie(self.buildURI(ids[0]), cf=cf) as scf:
            print("Using crazyflie " + str(ids[0]) + " as origin")

            print("Reading sensor data...")
            sweep_angle_reader = LighthouseSweepAngleAverageReader(
                scf.cf, self.angles_collected_cb)
            sweep_angle_reader.start_angle_collection()
            self.collection_event.wait()

            print("Estimating position of base stations...")
            estimator = LighthouseBsGeoEstimator()

            for id in sorted(self.sensor_vectors_all.keys()):
                average_data = self.sensor_vectors_all[id]
                sensor_data = average_data[1]
                rotation_bs_matrix, position_bs_vector = estimator.estimate_geometry(
                    sensor_data)
                is_valid = estimator.sanity_check_result(position_bs_vector)

                if is_valid:
                    geo = LighthouseBsGeometry()
                    geo.rotation_matrix = rotation_bs_matrix
                    geo.origin = position_bs_vector
                    geo.valid = True

                    if invert or invert2:
                        geo = self.invertBSCoord(geo, invert2)

                    geometries[id] = geo

                    # self.print_geo(id, geo.rotation_matrix,
                    #               geo.origin, is_valid)

                    # geoStore.append(position_bs_vector)

                else:
                    print("Warning: could not find valid solution for " + id + 1)

                print()

        if do_write:
            for id in ids:
                with SyncCrazyflie(self.buildURI(id), cf=cf) as scf:
                    self.write_event.clear()
                    print("Uploading geo data to CF " + str(id))
                    helper = LighthouseMemHelper(scf.cf)
                    helper.write_geos(geometries, self.write_done_cb)
                    self.write_event.wait()

        return geoStore

    def print_geo(self, bsid, rotation_cf, position_cf, is_valid):
        #print('Base station ' + bsid + ' geometry:')
        print('C-format')
        if is_valid:
            valid_c = 'true'
        else:
            valid_c = 'false'

        print('{.valid = ' + valid_c + ', .origin = {', end='')
        for i in position_cf:
            print("{:0.6f}, ".format(i), end='')

        print("}, .mat = {", end='')

        for i in rotation_cf:
            print("{", end='')
            for j in i:
                print("{:0.6f}, ".format(j), end='')
            print("}, ", end='')

        print("}},")

        print()
        print('python-format')
        print('geo = LighthouseBsGeometry()')
        print('geo.origin =', np.array2string(position_cf, separator=','))
        print('geo.rotation_matrix = [', end='')
        for row in rotation_cf:
            print(np.array2string(row, separator=','), end='')
            print(', ', end='')
        print(']')
        print('geo.valid =', is_valid)

    def buildURI(self, id):
        # This method has no checks and only works for ids less than 10 cause ceebs
        baseURI = 'radio://0/80/2M/E7E7E7E7'
        uri = baseURI + '0' + str(id)
        return uri

    def invertBSCoord(self, geo: LighthouseBsGeometry, invert2: bool) -> LighthouseBsGeometry:
        # Convert BS geo data to 4x4 numpy arrays -> TT and TR1
        rot = np.zeros((4, 4))
        tr = np.zeros((4, 4))
        rot[3][3] = 1
        tr[3][3] = 1

        for row in range(0, 3, 1):
            tr[row][3] = geo.origin[row]

            for element in range(0, 3, 1):
                rot[row][element] = geo.rotation_matrix[row][element]

                if element == row:
                    tr[row][element] = 1

        # Create numpy array for required rotation matrix
        if not invert2:
            invRot = np.array([[1, 0, 0, 0], [0, -1, 0, 0],
                              [0, 0, -1, 0], [0, 0, 0, 1]])
        else:
            # Do initial -42 deg rotation
            initialRot = np.array(
                [[0.743, 0, -0.669, 0], [0, 1, 0, 0], [0.669, 0, 0.743, 0], [0, 0, 0, 1]])
            rot = rot.dot(initialRot)
            a = 3.14159-(2*atan(geo.origin[1]/geo.origin[0]))
            invRot = np.array([[cos(a), -sin(a), 0, 0], [sin(a), cos(a), 0, 0],
                              [0, 0, 1, 0], [0, 0, 0, 1]])

        # Do T = TT.TR1
        t = tr.dot(rot)

        # Do T = TR2.T
        t = invRot.dot(t)

        # Extract new rotation matrix and position vector from T, set z co-ord and return
        newRot = [[], [], []]
        newOrigin = []

        for row in range(0, 3, 1):
            newOrigin.append(t[row][3])
            for element in range(0, 3, 1):
                newRot[row].append(t[row][element])

        # Fix z-axis at 8cm above the ground for inverted setups
        if not invert2:
            newOrigin[2] = 0.08
        else:
            newOrigin[2] = 0.43

        geo.rotation_matrix = newRot
        geo.origin = newOrigin

        print(newRot)
        print(newOrigin)

        return geo


def writeToCSV(geoStore):
    # Assumes the use of 2 base stations
    print("Creating bs_geometry.csv...")
    with open('bs_geometry.csv', 'w+', newline='') as csvfile:
        newdata = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
        newdata.writerow([geoStore[0][0]]+[geoStore[0][1]] +
                         [geoStore[0][2]])  # lighthouse 1
        newdata.writerow([geoStore[1][0]]+[geoStore[1][1]] +
                         [geoStore[1][2]])  # lighthouse 2

    with open('bs_geometry.csv', 'r', newline='') as csvfile:
        csvfile = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in csvfile:
            print(row[0], row[1], row[2])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--write", help="upload the calculated geo data to the Crazyflie", action="store_true")
    parser.add_argument(
        "--csv", help="write calculated geo data to .csv file", action="store_true")
    parser.add_argument(
        "--invert", help="use when LH deck is mounted under CF. Rotates base station co-ordinate systems around x-axis", action="store_true")
    parser.add_argument(
        "--invert2", help="use when LH deck is mounted under CF. Mirrors base station co-ordinate system in the y-plane", action="store_true")

    args = parser.parse_args()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Load ids from crazyflies.yaml to build URIs for writing
    rospack = rospkg.RosPack()
    yamlPath = rospack.get_path('crazyswarm_lnkd')+"/launch/crazyflies.yaml"

    with open(yamlPath, 'r') as ymlfile:
        cfg = yaml.load(ymlfile)

    ids = []
    for crazyflie in cfg["crazyflies"]:
        ids.append(int(crazyflie["id"]))

    # Estimate the lighthouse positions
    estimator = Estimator()
    geoStore = estimator.estimate(ids, args.write, args.invert, args.invert2)

    if args.csv:
        writeToCSV(geoStore)

    print("Base station geometry estimation complete!")
