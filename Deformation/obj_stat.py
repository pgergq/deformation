'''
Count vertices, minx-maxx, miny-maxy, minz-maxz, xdistance, ydistance, zdistance
in an obj file

Usage: obj_stat.py [-h] [-i INPUT_FILE]

Created on 2013.12.01.
@author: Gergely Petrik, EUJ9H7

'''

import argparse
import sys
import os.path
import re
from datetime import datetime

## Argument parser initialization and setup
parser = argparse.ArgumentParser();
parser.add_argument("-i", "--input", help="Input file", type=str, required=True)
args = parser.parse_args();

## Argument checking
if not os.path.exists(args.input):
    print("Error: Input file not found.")
    sys.exit(1)

else:
    first = 1
    count = 0
    for line in open(args.input, "r"):
            if line.startswith('#'): continue
            values = line.split()
            if not values: continue
            if values[0] == 'v':
                count+=1
                if first == 1:
                    minx = maxx = float(values[1])
                    miny = maxy = float(values[2])
                    minz = maxz = float(values[3])
                    first = 0
                else:
                    minx = min(minx, float(values[1]))
                    maxx = max(maxx, float(values[1]))
                    miny = min(miny, float(values[2]))
                    maxy = max(maxy, float(values[2]))
                    minz = min(minz, float(values[3]))
                    maxz = max(maxz, float(values[3]))
            else:
                break
    x = str(datetime.now()).replace(":","-").replace(" ","_")
    ou = open("stat_"+str(os.path.splitext(args.input)[0])+"_"+x+".txt", "w")
    ou.write("# of vertices: "+str(count)+"\n\n")
    ou.write("minx: "+str(minx)+"\n")
    ou.write("maxx: "+str(maxx)+"\n")
    ou.write("miny: "+str(miny)+"\n")
    ou.write("maxy: "+str(maxy)+"\n")
    ou.write("minz: "+str(minz)+"\n")
    ou.write("maxz: "+str(maxz)+"\n")
    
    print("# of vertices: "+str(count)+"\n\n")
    print("minx: "+str(minx)+"\n")
    print("maxx: "+str(maxx)+"\n")
    print("miny: "+str(miny)+"\n")
    print("maxy: "+str(maxy)+"\n")
    print("minz: "+str(minz)+"\n")
    print("maxz: "+str(maxz)+"\n")
    
    ou.close()
