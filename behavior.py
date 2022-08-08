#!/usr/bin/env python3
#
# Program that alternates between a number of behaviors for the robot arm. 
# as a part of the Three Stage Drawing Transfer, interactive artwork. 
#
# Robert Twomey - roberttwomey.com - 2022

import sys
import math
import time
import datetime
import random
import traceback
import threading
import cv2
import time
import numpy as np
import simplejson


# ==== STATES ====
CAMERA_NUM = 2

PERSON = 0
DRAW = 1
REST = 2
ASSESS_ERASED=3
ASSESS_DRAWN=4

robotState = ASSESS_ERASED

# order of operations
# 1 - scan for faces
# 2 - look for drawing
# 3 - add to drawing
# 4 - rest

FACE_TIMEOUT = 10.0 # how long does a person have to disappear for the system to reset
ENGAGEMENT_TIME = 10.0 # how long does a person have to be seen by the robot for the cycle to start
REST_TIMEOUT = 20.0 # how long do we rest when we are done drawing
ERASED_TIMEOUT = 5.0 # how soon do we start over when everything is erased
DRAWN_TIMEOUT = 15.0 # how long does the human have to draw

DRAWN_LENGTH = 10000.0 # perimeter at least 10kpx long to be adequate human drawing
ERASED_LENGTH = 1000.0 # perimeter for when it is erased


# joint angle descriptions of front look and down look
frontAngle = [0, 2.5, 0, 37.3, 0, -57.3, 0]

# downAngle = [0.8, -0.9, 0.8, 78.8, 0.1, 78.3, 1.5]
downAngle = [0.8, -1, 0.8, 77.3, 0.1, 76.9, 1.5] # closer
downPos = [467.8, 7, 569.1, -179.9, -1.5, 0]

variables = {}

# SIGGRAPH
params = {'speed': 100, 'acc': 2000, 'angle_speed': 75, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

closeSizeCutoff = 250.0

showDebug = True
verbose = False

# State
robotX = 424.0
robotY = 0
robotZ = 400.0
bStarted = False
startTime = 0
timeLastSeen = 0
bCloseFace = False
timeSeenClose = 0
bMadeDrawing = False

# Robot Params (speed etc.)
drawSpeed = 50
drawRadius = 0.0
drawAccel = 500
rapidSpeed = 600#500
rapidAccel = 1000


# timing of robot control signal
updateInterval = 0.1
tLastUpdated = time.time()

# ==== Setup Robot ====

"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.4.15')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)


# Register error/warn changed callback
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(connect_changed_callback)


def goZero(): 
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=[0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))


def lookDown(): 
    if arm.error_code == 0 and not params['quit']:
        # code = arm.set_servo_angle(angle=downAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        code = arm.set_servo_angle(angle=downAngle, speed=rapidSpeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)
        # code = arm.set_position(*downPos, speed=rapidSpeed, mvacc=rapidAccel, wait=True)       
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))

def lookForward():
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=frontAngle, speed=rapidSpeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))

# Rotation
if not params['quit']:
    params['angle_acc'] = 1145
if not params['quit']:
    params['angle_speed'] = 50
    # if params['quit']:
    lookForward()

# ==== Drawing Helpers and Parameters ====

# define key positions and mapping to rectangle on page
rest = [250, 0, 120, 180, 0, 0]

zheight = 112.1
liftHeight = 10.0#15.0
topleft = [222.7, -256.0, zheight, 180, 0, 0]
topleftup = list(topleft)
topleftup[2]+=liftHeight
botleft = [557.2, -256.0, zheight, 180.0, 0.0, 0.0]
botright = [563.1, 250.0, zheight, 180.0, 0.0, 0.0]
topright = [240.7, 250.0, zheight, 180.0, 0.0, 0.0]


# Buffer to store drawing
# colors
WHITE = (255, 255, 255)
BLUE = (255, 255, 0)
BLACK = (0, 0, 0)

# blank image 
outheight = 720
outwidth = 1280
penpathImage = np.zeros((outheight, outwidth, 3), np.uint8)
cv2.rectangle(penpathImage, (0,0), (outheight, outwidth), WHITE, cv2.FILLED)
linewidth = 3

# Geometry Transformation and Helpers

def calcTransformed(matrix, p):
    px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    p_after = (int(px), int(py))
    return p_after

def calcArmPos(matrix, p):
    p_after = calcTransformed(matrix, p)
    position = [p_after[0], p_after[1], zheight, 180.0, 0.0, 0.0] # no radius
    # position = [p_after[0], p_after[1], zheight, 180.0, 0.0, 0.0, 1.0] # radio, I plan motion (?)
    return position

# calculate transform to robot arm space
points1=np.array([[-0.176,0], [-0.176, 1.0], [1.176, 1.0], [1.176, 0]])
points2=np.array([topleft[:2], botleft[:2], botright[:2], topright[:2]])

# reference https://www.pythonpool.com/cv2-findhomography/
H, status = cv2.findHomography(points1, points2)
# print(H, status)


# ==== Drawing File Loading

files = [
    "../data/frame00572.json",
    "../data/frame00640.json",
    "../data/frame01463.json",
    "../data/GAN_sample02.json",
    "../data/sample01.json",
    "../data/frame00237.json"
    ]

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

            if count > 2:
                polys.append(this_path)

    return polys


def sortPaths(paths, longest=False):
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

                if verbose:
                    print("  testing {0}, len1: {1:.2f}\tlen2: {2:.2f}\tmin: {3:.2f}".format(i, beginlen, endlen, minlen))

        # we have our victor
        if verbose:
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

    if longest:
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


# ==== OpenCV / Vision ====

# To capture video from webcam
cap = cv2.VideoCapture(CAMERA_NUM) # choose the proper camera number
capWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
capHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Load the face detection cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# load text
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
thickness              = 1
lineType               = 2

tiltAng = 0
panAng = 5

# OpenCV helpers
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    # from here http://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/

    v = np.median(image)
 
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edge = cv2.Canny(image, lower, upper)
 
    # return the edge image
    return edge

def detectContours(img):
    # detect contours
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.blur(imgray, (3,3))
    edged = auto_canny(blurred, 0.3)
    # ret, thresh = cv2.threshold(imgray, 156, 255, 0)
    ret, thresh = cv2.threshold(edged, 64, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # _, binary = cv2.threshold(blurred, 32, 255, cv2.THRESH_BINARY)
    # contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # find total perimeter
    total = 0
    for cntr in contours:
        area = cv2.contourArea(cntr)
        perimeter = cv2.arcLength(cntr, True)
        total+=perimeter

    return contours, total


# ==== MAIN LOOP OF PROGRAM ====

while True:

    if robotState == DRAW: 
        # drawing is a blocking behavior

        cv2.rectangle(penpathImage, (0,0), (outheight, outwidth), (255, 255, 255), cv2.FILLED)
        cv2.imshow('img', penpathImage)

        # cv2.putText(penpathImage,"DRAWING", (10, 210), 
        #     font, 
        #     fontScale,
        #     fontColor,
        #     thickness,
        #     lineType)

        arm.set_position(*rest, speed=rapidSpeed, wait=True)
        arm.set_position(*topleftup, speed=rapidSpeed, wait=True)

        # points = [ (0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]
        # path=[[-0.176,0], [-0.176, 1.0], [1.176, 1.0], [1.176, 0], [-0.176,0]]

        infile = random.choice(files)

        print(f"======== DRAWING FILE {infile} ========")

        unsorted_paths = readJSON(infile, 908, 600)#681)
        paths = sortPaths(unsorted_paths)

        pngfile = infile.split(".json")[0]+"_paths.png"
        renderAsImage(paths, pngfile, 1024, 1024)

        new_paths=[]
        for path in paths:
            this_path = [] 
            for point in path:
                position = calcArmPos(H, point)
                this_path.append(position)
            new_paths.append(this_path)

        for path in new_paths:
            
            # move to start with pen in air
            start_point = list(path[0])
            start_point[2] += liftHeight
            print("path started...", end="")
            sys.stdout.flush()
            arm.set_position(*start_point, speed=drawSpeed, mvacc=drawAccel, wait=False)

            # send points and make drawing
            for point in path:
                arm.set_position(*point, wait=False, mvacc=drawAccel, radius=drawRadius) # MoveArcLine with interpolation, linear arc motion
                # print("moving to %s" % position)

            # lift pen in air before starting next path
            end_point = list(path[-1])
            end_point[2] += liftHeight
            # arm.set_position(*end_point, speed=params['speed'], wait=True)
            # arm.set_position(*end_point, speed=rapidSpeed, mvacc=drawAccel, wait=True, radius=drawRadius)
            arm.set_position(*end_point, speed=drawSpeed, mvacc=drawAccel, wait=True)

            print("path ended, sent {} points".format(len(path)))#: {}".format(end_point))
            sys.stdout.flush()

        # lookForward()
        # robotState = PERSON

        # # go to rest pos and wait
        bStarted = False
        robotState = REST
        
        # admire your handiwork and wait for someone to erase it
        # bStarted = False
        # robotState = ASSESS_ERASED
    
    elif robotState == REST: 
        if not bStarted:
            print("==== RESTING ====")
            bStarted = True
            startTime = time.time()
            goZero()

        if time.time() - startTime > REST_TIMEOUT:
            lookForward()
            robotState = ASSESS_ERASED
            bStarted = False
            timeLastSeen = time.time()
            startTime = time.time()
    else:
        
        # Read the frame
        _, img = cap.read()
    
        if img is not None:
            
            if robotState == ASSESS_ERASED:
                
                # look down to see if they have erased the drawing
                if not bStarted:
                    lookDown()
                    bStarted = True
                    print("==== ASSESS_ERASED ====")
                    bErased = False

                # flip image if we are looking at the paper
                img = cv2.flip(img, -1)

                contours, total_perimeter = detectContours(img)

                cv2.putText(img,"{} contours of length {:.2f}".format(len(contours), total_perimeter), (10, 170),
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)

                cv2.drawContours(img, contours, -1, (0,255,0), 3)

                if total_perimeter < ERASED_LENGTH:
                    if not bErased: 
                        bErased = True
                        startTime = time.time()
                else: 
                    bErased = False

                if time.time() - startTime > ERASED_TIMEOUT and bErased:
                    robotState = PERSON
                    lookForward()
                    bStarted = False

                    # to oscillate between DRAW and ERASE to prompt machine to DRAW
                    # robotState = DRAW
                    # bStarted = False
                    # startTime = time.time()

                    # erase the image while drawing
                    # cv2.rectangle(img, (0,0), (int(capWidth), int(capHeight)), (255, 255, 255), cv2.FILLED)
                    
            elif robotState == ASSESS_DRAWN:
                
                # look down
                if not bStarted:
                    # lookForward()
                    lookDown()
                    bStarted = True
                    print("==== ASSESS_DRAWN ====")
                    bDrawn = False

                # flip image if we are looking at the paper
                img = cv2.flip(img, -1)

                contours, total_perimeter = detectContours(img)

                cv2.putText(img,"{} contours of length {:.2f}".format(len(contours), total_perimeter), (10, 170),
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)

                cv2.drawContours(img, contours, -1, (0,255,0), 3)

                if total_perimeter > DRAWN_LENGTH:
                    if not bDrawn: 
                        bDrawn = True
                        startTime = time.time()
                else: 
                    bDrawn = False

                if time.time() - startTime > ERASED_TIMEOUT and bDrawn:
                    robotState = DRAW
                    bStarted = False
                    startTime = time.time()


            elif robotState == PERSON:
    
                # Convert to grayscale
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                # Detect the faces
                faces = face_cascade.detectMultiScale(gray, 1.1, 4, minSize=(150,150), maxSize=(600, 600))
                # Draw the rectangle around each face
                
                facecount = 0
                maxSize = 0
                maxIndex = -1
                maxLoc = None

                for (x, y, w, h) in faces:

                    if showDebug:
                        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                        cv2.putText(img,"{} x {}".format(w,h), (x,y), 
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)
                    
                    # check how close the face is            
                    if w > maxSize: 
                        maxSize = w
                        maxIndex = facecount
                        maxLoc = (x, y, w, h)
                    facecount += 1
                
                if (facecount > 0) and (maxSize > closeSizeCutoff):

                    if not bStarted:
                        print("==== PERSON DETECTED ====")
                        startTime = time.time()
                        bStarted = True

                    # print(x, maxSize, closeSizeCutoff)
                    # calculate distance of first face from center of image
                    (x, y, w, h) = maxLoc
                    offsetX = (0.5*capWidth-(x+w*0.5))/capWidth
                    offsetY = (0.5*capHeight-(y+h*0.5))/capHeight
                    
                    scale = 7.0
                    dTilt = -1.0*scale*offsetY
                    dPan = scale*offsetX
                    
                    # add front back moves
                    #print(dTilt, dPan)
                    # print('* position:', arm.position)

                    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 10)


                    if time.time() - tLastUpdated > updateInterval:
                        # arm.set_position(pitch=dTilt, roll=dPan, relative=True, speed=500, mvacc=2000, wait=False)
                        # better motion
                        ret = arm.set_position_aa(axis_angle_pose=[0, 0, 0, 0, dTilt, dPan], speed=500, relative=True, wait=False)
                        tLastUpdated = time.time()
                    
                    # # CHECK IF WE ARE CLOSE
                    # if maxSize > 400:
                    #     # keep track that we are seeing a close face
                    #     if not bCloseFace:
                    #         bCloseFace = True
                    #         timeSeenClose = time.time()

                    #     # if time.time() - timeSeenClose > timeLookClose:
                    #     #     # switch to look at ASSESS
                    #     #     robotState = ASSESS
                    #     #     bStarted = False
                    #     #     bCloseFace = False

                    timeLastSeen = time.time()

                    if time.time() - startTime > ENGAGEMENT_TIME:
                        # switch to look at ASSESS
                        robotState = ASSESS_DRAWN
                        bStarted = False
                        bCloseFace = False        
                    
                else:
                    # no adequate faces
                    timeElapsed = time.time() - timeLastSeen

                    cv2.putText(img, "{:.2f}".format(timeElapsed), (10, 170), 
                        font, 
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)

                    if timeElapsed > 1.0 and timeElapsed < 5.0:
                        
                        # relax back

                        # print("relaxing to front")
                        cv2.putText(img,"RELAXING", (10, 210), 
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)

                        if time.time() - tLastUpdated > updateInterval:

                            # relax to front position
                            currAngle = arm.angles
                            weight=0.9
                            # weight = 0.25
                            destAngle = [(1.0-weight)*currAngle[i]+weight*frontAngle[i] for i in range(len(frontAngle))]
                            arm.set_servo_angle(angle=destAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=False, radius=-1.0)

                            tLastUpdated = time.time()

                    # if time.time() - timeSeenClose > timeLookClose:
                    if time.time() - timeSeenClose > FACE_TIMEOUT:
                        # switch to look at ASSESS
                        robotState = PERSON
                        bStarted = False
                        bCloseFace = False

        cv2.putText(img,"state: {} time: {:.2f}(s)".format(robotState, time.time() - startTime), (10, 70), 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)

        if img is not None:
            # Display video
            cv2.imshow('img', img)

    # Stop if escape key is pressed
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break

# Release the VideoCapture object
cap.release()

# restore robot arm and disconnect

# return to forward position
arm.set_servo_angle(angle=frontAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
