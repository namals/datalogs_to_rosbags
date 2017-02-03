#!/usr/bin/env python

#    Script to convert pointcloud data in csv format to ROS's PointCloud2 format
#    and write to a rosbag
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
from sensor_msgs.msg import PointCloud2
import rosbag
import csvutils


# utility to read a point cloud from csv and generate a PointCloud2 msg
def generatePointCloud2(xyzreader, fileId):    
    cloud_stamp = 0
    points = []
    i = 0
    for row in xyzreader:
        if i == 0:
            i+=1            
            continue
        if i == 1:
            cloud_stamp = row[0]
        x = float(row[1])
        y = float(row[2])
        z = float(row[3]) 
        p = [x, y, z]
        points.append(p)
        
    header = Header()        
    header.frame_id = "map"
    if cloud_interval == 0:
        times = cloud_stamp.split(".")
        header.stamp.secs = int(times[0])
        header.stamp.nsecs = int(times[1])
    else:
        header.stamp.secs = 1 + cloud_interval*fileId
        header.stamp.nsecs = 0
    pc = pc2.create_cloud_xyz32(header, points)
    return pc

# check if directory is provided
if len(sys.argv) < 2:
    print "Directory to read csv files is not provided! Quitting...."
    sys.exit(0)

cloud_interval = 0
if len(sys.argv) == 3:
    cloud_interval = float(sys.argv[2])

directory = sys.argv[1]
print "Reading PointCloud csv files from ", directory

# generate the bag file with the same name as the parent directory
outbagName = csvutils.getDirectoryName(directory)

# change directory to this directory
os.chdir(directory)
bag = rosbag.Bag(outbagName+'.bag', 'w')

files = csvutils.getFileListInDirectoryWithPattern(directory, "PointCloud*.csv")
for i in range(len(files)):
    print "Reading file : ", files[i]
    with open(files[i], 'rb') as csvfile:
        xyzreader = csv.reader(csvfile)
        pc = generatePointCloud2(xyzreader, i)
        try:
            bag.write('cloud_in', pc)
        except IOError as e:
            print "Something went wrong during writing to bag file, quitting"
            bag.close()
            sys.exit(0)

## get csv files sorted according to their numerical suffix
#fileId = 1
#for file in sorted(glob.glob("PointCloud*.csv"), key=csvutils.numericalSort):
#    print file
#    with open(file, 'rb') as csvfile:
#        xyzreader = csv.reader(csvfile)
#        pc = generatePointCloud2(xyzreader, fileId)
#        fileId += 1
#        try:
#            bag.write('cloud_in', pc)
#        except IOError as e:
#            print "Something wrong in writing to bag, quitting"
#            bag.close()
#            sys.exit(0)

print "Successfully written pointclouds to bag"
bag.close();
sys.exit(0)
