#!/usr/bin/env python

import glob, os, sys
import re
import csv
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import rosbag

# utility to sort files w.r.t to their integer suffix
numbers = re.compile('(\d+)')
def numericalSort(value):
    parts = numbers.split(value)
    return int(parts[1])

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
# change directory
os.chdir(directory)
cwd = os.getcwd()
dirnames = cwd.split('/')
outbagName = dirnames[len(dirnames)-1]
print "Current directory is : ", outbagName
bag = rosbag.Bag(outbagName+'.bag', 'w')

# get csv files sorted according to their numerical suffix
fileId = 1
for file in sorted(glob.glob("PointCloud*.csv"), key=numericalSort):
    print file
    with open(file, 'rb') as csvfile:
        xyzreader = csv.reader(csvfile)
        pc = generatePointCloud2(xyzreader, fileId)
        fileId += 1
        try:
            bag.write('cloud_in', pc)
        except IOError as e:
            print "Something wrong in writing to bag, quitting"
            bag.close()
            sys.exit(0)

print "Successfully written pointclouds to bag"
bag.close();
sys.close(0)
            
            
