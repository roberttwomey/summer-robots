#!/usr/local/bin/python3

# Simple text programs for python control of xARM
# reference for https://github.com/xArm-Developer/xArm-Python-SDK

from xarm.wrapper import XArmAPI
import time
import sys
import cv2
import simplejson
import numpy as np

bTest = False
simulate = False
liftHeight = 15.0
debug = False


# define key positions and mapping to rectangle on paper

rest = [250, 0, 120, 180, 0, 0]
zheight = 112.1
topleft = [222.7, -256.0, zheight, 180, 0, 0]
topleftup = list(topleft)
topleftup[2]+=liftHeight
botleft = [557.2, -256.0, zheight, 180.0, 0.0, 0.0]
botright = [563.1, 250.0, zheight, 180.0, 0.0, 0.0]
topright = [240.7, 250.0, zheight, 180.0, 0.0, 0.0]

# Create Transform
# points1=np.array([[0.0,0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]])
points1=np.array([[-0.176,0], [-0.176, 1.0], [1.176, 1.0], [1.176, 0]])
points2=np.array([topleft[:2], botleft[:2], botright[:2], topright[:2]])


# ==== Helper Functions ====

def calcTransformed(matrix, p):
    px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    p_after = (int(px), int(py))
    return p_after

def calcArmPos(matrix, p):
    p_after = calcTransformed(matrix, p)
    position = [p_after[0], p_after[1], zheight, 180.0, 0.0, 0.0]
    return position

# polygon loading / sorting
def readJSON(filename, width=224.0, height=224.0):
    with open (filename, "r") as myfile:
        data = myfile.read()

    pydata = simplejson.loads(data)
    # print(len(pydata))
    minlen = 0.00001

    paths_out = []
    count = 0
    for path in pydata:
        thispath = []
        bFirstPoint=True
        for point in path:
            # print(point)
            # BAD TO HARDCODE THIS SCALE FACTOR
            x = point['x']/width
            y = point['y']/height*0.75+0.125
            # print(x,y)
            pos = np.array([x, y])
            if bFirstPoint:
                lastpos = pos
                bFirstPoint = False

            if np.linalg.norm(pos-lastpos) < minlen:
                continue
            else: 
                lastpos = pos
                thispath.append((x, y))
            # thispath.append((x, y))
            count+=1
        
        if(len(thispath) > 5):
            paths_out.append(thispath)
    
    print("parsed {} paths with {} points in json file {}".format(len(pydata), count, filename))
    return paths_out

def readGeoJSON(filename):
    with open (filename, "r") as myfile:
        data = myfile.read()

    pydata = simplejson.loads(data)

    polys = [] 
    minlen = 0.01
    # maxlen = 0.25
    bFirstPoint = True

    for feat in pydata['features']:

        coords = feat['geometry']['coordinates']
        # print(len(coords))
        pcount = 0
        for path in coords:
            # print("{}: {} of {}".format(len(path), pcount, len(coords)))
            this_path = []
            count = 0
            pcount+=1
            for point in path:
                
                x = point[0]/680.0
                y = 0.875-(point[1]/512.0*0.752)

                pos = np.array([x, y])

                if bFirstPoint:
                    lastpos = list(pos)
                    bFirstPoint = False
                    this_path.append((x, y))
                    count+=1
                else:
                    dist = np.linalg.norm(pos-lastpos)

                    if dist < minlen:
                        continue
                    else: 
                        lastpos = list(pos)
                        this_path.append((x, y))
                        count+=1

            if count > 3:
                polys.append(this_path)

    return polys


def sortPaths(paths):
    """
    Starting at 0,0, sorts all polys in array by nearest neighor,
    flipping the beginning and ends if necessary.

    Returns sorted poly array.
    """
    
    print("starting sort...", end="")
    sys.stdout.flush()

    sortedpaths = []
    used = [ False ] * len(paths)    
    last = [0., 0.]
    done = False
    count = 0

    while not done:
        minlen = 99999
        closest = -1
        flipped = None
        
        for i, path in enumerate(paths):
            #print(i, used[i])
            
            if not used[i]:
                begin = paths[i][0] # first coordinate
                end = paths[i][-1] # last coordinate
                
                beginlen = np.linalg.norm(np.array(begin)-np.array(last))
                endlen = np.linalg.norm(np.array(end)-np.array(last)) 
                if beginlen < minlen:
                    closest = i
                    flipped = False
                    minlen = beginlen

                if endlen < minlen:
                    closest = i
                    flipped = True
                    minlen = endlen

                if debug:
                    print("  testing {0}, len1: {1:.2f}\tlen2: {2:.2f}\tmin: {3:.2f}".format(i, beginlen, endlen, minlen))

        # we have our victor
        if debug:
            print("best {} flipped {} {:.2f}".format(closest, flipped, minlen))
        if flipped:
            sortedpaths.append(paths[closest][::-1])
        else:
            sortedpaths.append(paths[closest])

        used[closest] = True
        last = sortedpaths[-1][-1]
        count = count + 1
        sys.stdout.flush()
        
        if count == len(paths):
            done = True

    print("done. sorted {0} paths".format(count))

    sortedpaths.sort(key=len, reverse=True)
    
    return sortedpaths

def renderAsImage(paths, filename, outwidth, outheight):
    # colors
    WHITE = (255, 255, 255)
    BLUE = (255, 255, 0)
    BLACK = (0, 0, 0)

    outwidth *= 4
    outheight *= 4

    # blank image 
    img = np.zeros((outheight, outwidth, 3), np.uint8)

    cv2.rectangle(img, (0,0), (outheight, outwidth), WHITE, cv2.FILLED)
    linewidth = 3

    lastpoint = (0, 0)
    color=BLUE
    for path in paths:
        for point in path:
            cv2.line(img, (int(lastpoint[0]*outwidth), int(lastpoint[1]*outwidth)), (int(point[0]*outwidth), int(point[1]*outheight)), color, linewidth)
            lastpoint = list(point)
            color=BLACK
        color=BLUE

    print("Saving image {0}".format(filename))
    cv2.imwrite(filename, img)

# ==== Wrangle pen paths ====

# recpoints, bounds = readDrawingRecording("data/3891_0911092146.txt")
# unsorted_paths = readJSON("../data/CLIPassoGAN_sample02_segs.json")
# unsorted_paths = readGeoJSON("../data/GAN_sample02.json")
# unsorted_paths = readGeoJSON("../data/sample01.json")

# print("Number of arguments: ", len(sys.argv))
# print("The arguments are: " , str(sys.argv))
# exit()

if len(sys.argv) > 1:
    infile = sys.argv[1]
    pngfile = sys.argv[1].split(".json")[0]+"_paths.png"

# unsorted_paths = readJSON("../data/sample01_ink_flatten.json", 908, 681)
# paths = sortPaths(unsorted_paths)
# renderAsImage(paths, "../data/sample01_ink_flatten.png", 1024, 1024)
unsorted_paths = readJSON(infile, 908, 681)
paths = sortPaths(unsorted_paths)
renderAsImage(paths, pngfile, 1024, 1024)


if bTest:
    exit()
# ==== transform path to robot arm space ====

# reference https://www.pythonpool.com/cv2-findhomography/
H, status = cv2.findHomography(points1, points2)
# print(H, status)

new_paths = []

for path in paths:
    this_path = [] 
    for point in path:
        position = calcArmPos(H, point)
        this_path.append(position)
    new_paths.append(this_path)

# print(new_paths)

# ==== Initialize Robot Arm ====

arm = XArmAPI('192.168.4.15')
arm.connect()
# arm.set_pause_time(0.2)
arm.set_simulation_robot(on_off=simulate)


# Position Arm

# joint angle locations
lookdown = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
lookforward = [0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0]

thisspeed = 400 #200 #120 # 350

# go to look
code = arm.set_servo_angle(angle=lookforward, speed=20, mvacc=500, wait=True, radius=-1.0)

arm.set_position(*rest, speed=thisspeed, wait=True)
arm.set_position(*topleftup, speed=thisspeed, wait=True)


# ==== DO ACTUAL DRAWING ====

for path in new_paths:

    n = 5#10#25#100
    count = 0
    
    # move to start with pen in air
    start_point = list(path[0])
    start_point[2] += liftHeight
    print("path started: {}".format(start_point))
    arm.set_position(*start_point, speed=thisspeed, wait=True)

    for i in range(0, len(path), n):
        streampoints = path[i:i+n]

        count += n
        print("drawing {} points out of {}...".format(count, len(path)))
        sys.stdout.flush()

        arm.move_arc_lines(streampoints, speed=100, times=1, wait=False)

    # lift pen in air before starting next path
    end_point = list(path[-1])
    end_point[2] += liftHeight
    print("path ended: {}".format(end_point))
    arm.set_position(*end_point, speed=thisspeed, wait=True)

print("done.")

code = arm.set_servo_angle(angle=lookforward, speed=20, mvacc=500, wait=True, radius=-1.0)
