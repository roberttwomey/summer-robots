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

# ==== STATES ====
CAMERA_NUM = 2

FACE = 0
PAPER = 1
DRAW = 2
REST = 3

# order of operations
# 1 - scan for faces
# 2 - look for drawing
# 3 - add to drawing
# 4 - rest

FACE_INTERVAL = 5.0
REST_INTERVAL = 10.0
timeLookClose = 1.5

# joint angle descriptions of front look and down look
frontAngle = [0, 2.5, 0, 37.3, 0, -57.3, 0]
downAngle = [0.8, -0.9, 0.8, 78.8, 0.1, 78.3, 1.5]
downPos = [467.8, 7, 569.1, -179.9, -1.5, 0]

variables = {}

# SIGGRAPH
params = {'speed': 100, 'acc': 2000, 'angle_speed': 75, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

closeSizeCutoff = 250.0

showDebug = True

# State
robotBehavior = FACE

robotX = 424.0
robotY = 0
robotZ = 400.0
bStarted = False
startTime = 0
timeLastSeen = 0
timeLookPaper = 5.0
bCloseFace = False
timeSeenClose = 0
bMadeDrawing = False

# Robot Params (speed etc.)
drawSpeed = 50
rapidSpeed = 500
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

# define key positions and mapping to rectangle on paper
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

# ==== Setup OpenCV / Vision ====

# To capture video from webcam. 
cap = cv2.VideoCapture(CAMERA_NUM) # choose the proper camera number
# cap = cv2.VideoCapture(1) # facetime camera
# To use a video file as input 
# cap = cv2.VideoCapture('filename.mp4')

capWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
capHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Load the cascade
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

while True:

    if robotBehavior == DRAW: 
        # drawing is a blocking behavior

        # cv2.rectangle(penpathImage, (0,0), (outheight, outwidth), WHITE, cv2.FILLED)
        # cv2.imshow('img', penpathImage)

        # cv2.putText(penpathImage,"DRAWING", (10, 210), 
        #     font, 
        #     fontScale,
        #     fontColor,
        #     thickness,
        #     lineType)

        arm.set_position(*rest, speed=rapidSpeed, wait=True)
        arm.set_position(*topleftup, speed=rapidSpeed, wait=True)

        # points = [ (0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]
        path=[[-0.176,0], [-0.176, 1.0], [1.176, 1.0], [1.176, 0], [-0.176,0]]

        # render our image here
        for point in path:
            position = calcArmPos(H, point)
            arm.set_position(*position, speed=400, wait=True)
            # arm.set_position(*position, speed=drawSpeed, wait=True)
            print("moving to %s" % position)

        lookForward()
        # robotBehavior = FACE

        # go to rest pos and wait

        bStarted = False
        robotBehavior = REST
        startTime = time.time()
        goZero()
        
    
    elif robotBehavior == REST: 
        if time.time() - startTime > REST_INTERVAL:
            lookForward()
            robotBehavior = FACE
            bStarted = False
            timeLastSeen = time.time()
    else:
        
        # Read the frame
        _, img = cap.read()
    
        if img is not None:
            
            if robotBehavior == PAPER:
                
                # look down
                if not bStarted:
                    startTime = time.time()
                    # lookForward()
                    lookDown()
                    bStarted = True

                # flip image if we are looking at the paper
                img = cv2.flip(img, -1)

                cv2.rectangle(img, (160, 0), (1760, 1080), (255, 255, 0), 10)

                # look for changes on paper
                # decide when it is empty again
                # DECIDE IF IT IS EMPTY THEN: 
                # - Y add a drawing
                # - N engage the viewer

                if time.time() - startTime > timeLookPaper:
                    # robotBehavior = FACE
                    # lookForward()
                    robotBehavior = DRAW
                    bStarted = False
                    startTime = time.time()

            elif robotBehavior == FACE:
    
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

                    
                    # CHECK IF WE ARE CLOSE
                    if maxSize > 400:
                        # keep track that we are seeing a close face
                        if not bCloseFace:
                            bCloseFace = True
                            timeSeenClose = time.time()

                        # if time.time() - timeSeenClose > timeLookClose:
                        #     # switch to look at paper
                        #     robotBehavior = PAPER
                        #     bStarted = False
                        #     bCloseFace = False

                    # DOESN'T WORK
                    # dollyX = offsetX
                    # dollyZ = offsetY
                    # ret = arm.set_position_aa(axis_angle_pose=[dollyX, 0, dollyZ, 0, dTilt, dPan], speed=700, relative=True, wait=False, radius=1.0)

                    # arm.set_tool_position(pitch=dTilt, wait=False)
                    # move j1
                    # code = arm.set_servo_angle(servo_id=1, angle=dPan, relative=True, is_radian=False, wait=True)
                    # print(code)

                    timeLastSeen = time.time()
                    tLastUpdate = time.time()
                    
                else:

                    timeElapsed = time.time() - timeLastSeen
                    cv2.putText(img, "{:.2f}".format(timeElapsed), (10, 170), 
                        font, 
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)

                    if timeElapsed > 1.0 and timeElapsed < 5.0:
                        
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


                    if time.time() - timeSeenClose > timeLookClose:
                        # switch to look at paper
                        robotBehavior = PAPER
                        bStarted = False
                        bCloseFace = False


                    # elif timeElapsed >= 5.0:
                    #     # switch to draw
                    #     lookForward()
                    #     robotBehavior = DRAW
                    #     bStarted = False
                        

                    # if bStarted: 
                    #     bStarted = False
                    #     timeLastSeen = time.time()

                    # if not bStarted and time.time()-timeLastSeen > 3.0:
                    #     # lost face, reset timer
                    #     startTime = time.time()

                    #     # move back to center
                    #     # lookForward()

                    #     # relax to front position
                    #     currAngle = arm.angles
                    #     weight=0.05
                    #     destAngle = [(1.0-weight)*currAngle[i]+weight*frontAngle[i] for i in range(len(frontAngle))]
                    #     arm.set_servo_angle(angle=destAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=False, radius=-1.0)            

                # if time.time() - startTime > timeLookFace: 
                #     robotBehavior = PAPER
                #     bStarted = False

        cv2.putText(img,"state: {} time: {:.2f}".format(robotBehavior, time.time() - startTime), (10, 70), 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)


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


## LEFTOVERS

# ISEA
# params = {'speed': 800, 'acc': 2000, 'angle_speed': 75, 'angle_acc': 1000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False} # ISEA
# params = {'speed': 500, 'acc': 2000, 'angle_speed': 1000, 'angle_acc': 5000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
# params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

# frontAngle = [0.1, -34.9, -0.1, 1.6, 0, -63.5, 0.1]
# frontAngle = [0.2, 4.7, -0.2, 39.1, 0, -60.0]
# downAngle = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
# downPos = [467.8, 0.1, 589.1, -179.9, -1.5, 0]
# downAngle = [-0.2, -0.4, 0, 92.7, 1.1, 81.1 -0.2]
# downAngle = [0, 1.8, 0, 100, 0.1, 96.7, 0]
