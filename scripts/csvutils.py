#!/usr/bin/env python

#    Utils for reading and processing Comma Separated Value (CSV) files
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

import re
import os
import glob

def getDirectoryName(dirFullPath):
    dirnames = dirFullPath.split('/')
    return dirnames[len(dirnames)-1]

# utility to sort files w.r.t to their integer suffix
numbers = re.compile('(\d+)')
def numericalSort(value):
    parts = numbers.split(value)
    return int(parts[1])

def getFileListInDirectoryWithPattern(directory, pattern="PointCloud*.csv"):
    print "Finding files from ", directory, " with the name pattern : ", pattern
    # change directory
    os.chdir(directory)
    cwd = os.getcwd()    
    files = []
    for file in sorted(glob.glob(pattern), key=numericalSort):
        files.append(file)
    return files
