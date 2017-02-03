#!/usr/bin/env python

#    Script to convert pointcloud data in csv format to ROS's PointCloud2 format
#    and write to a rosbag as a single PointCloud2
#    Copyright (C) 2017  Namal Senarathne, namal.senarathne@gmail.com
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys, os
import csv
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import rosbag
import csvutils

# given a csv object, read the rows, extract point, add to points list
def updatePoint32List(xyzreader, points):
    i = 0
    for row in xyzreader:
        if i == 0:
            i+=1
            continue
        x = float(row[1])
        y = float(row[2])
        z = float(row[3])
        p = [x, y, z]
        points.append(p)

# given a list of points, create a sensor_msgs.PointCloud2
def generatePointCloud2(points):
    header = Header()
    header.frame_id = "map"
    header.stamp.secs = 1
    header.stamp.nsecs = 0
    pc = pc2.create_cloud_xyz32(header, points)
    return pc

# check if directory is provided
if len(sys.argv) < 2:
    print "Directory to read csv files is not provided! Quitting...."
    sys.exit(0)

path = sys.argv[1]
print "Reading PointCloud csv files from ", path

# generate the bag file with the same name as the parent directory
outbagName = csvutils.getDirectoryName(path)

# change directory to this directory
os.chdir(path)
bag = rosbag.Bag(outbagName+'_single.bag', 'w')

files = csvutils.getFileListInDirectoryWithPattern(path, "PointCloud*.csv")

points = []
for i in range(len(files)):
    print "Reading file : ", files[i]
    with open(files[i], 'rb') as csvfile:
        xyzreader = csv.reader(csvfile)
        updatePoint32List(xyzreader, points)

pc = generatePointCloud2(points)
try:
    bag.write('cloud_in', pc)
except IOError as e:
    print "Something went wrong during writing to bag file, quitting"
    bag.close()
    sys.exit(0)

bag.close()
sys.exit(0)
