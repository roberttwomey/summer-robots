#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
"""
import sys
import math
import time
import datetime
import random
import traceback
import threading
import cv2
import time

# ==== Configuration ====
FACE = 0
PAPER = 1
DRAW = 2

# frontAngle = [0.1, -34.9, -0.1, 1.6, 0, -63.5, 0.1]
# frontAngle = [0.2, 4.7, -0.2, 39.1, 0, -60.0]
# downAngle = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
# downPos = [467.8, 0.1, 589.1, -179.9, -1.5, 0]
frontAngle = [0, 2.5, 0, 37.3, 0, -57.3, 0]
downAngle = [-0.2, -0.4, 0, 92.7, 1.1, 81.1 -0.2]
downPos = [467.8, 7, 569.1, -179.9, -1.5, 0]

variables = {}
params = {'speed': 800, 'acc': 2000, 'angle_speed': 500, 'angle_acc': 1000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
# params = {'speed': 500, 'acc': 2000, 'angle_speed': 1000, 'angle_acc': 5000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
# params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}


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
timeLookFace = 20.0
timeLookPaper = 30.0
bCloseFace = False
timeSeenClose = 0
timeLookClose = 5.0

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

def lookDown(): 
    if arm.error_code == 0 and not params['quit']:
        # code = arm.set_servo_angle(angle=downAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        code = arm.set_position(*downPos, speed=params['speed'], wait=True)       
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))

def lookForward():
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=frontAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
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

# ==== Setup OpenCV / Vision ====

# To capture video from webcam. 
cap = cv2.VideoCapture(0)
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

    # Read the frame
    _, img = cap.read()

    if robotBehavior == DRAW: 
        
        # render our image here
        pass

    else:
        
        if img is not None:
            
            if robotBehavior == PAPER:
                # look down
                if not bStarted:
                    startTime = time.time()
                    lookForward()
                    lookDown()
                    bStarted = True

                cv2.rectangle(img, (160, 0), (1760, 1080), (255, 255, 0), 10)

                # look for changes on paper
                # decide when it is empty again

                if time.time() - startTime > timeLookPaper:
                    robotBehavior = FACE
                    lookForward()
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
                    
                    dTilt = -3.0*offsetY
                    dPan = 3.0*offsetX
                    
                    # accumulate tilt and Pan
                    # tiltAng += -0.05 * offsetY
                    # panAng += 0.042 * offsetX

                    # add front back moves
                    #print(dTilt, dPan)
                    # print('* position:', arm.position)

                    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 10)

                    # arm.set_position(pitch=dTilt, roll=dPan, relative=True, speed=500, mvacc=2000, wait=False)
                    
                    # better motion
                    ret = arm.set_position_aa(axis_angle_pose=[0, 0, 0, 0, dTilt, dPan], speed=700, relative=True, wait=False, radius=1.0)
                    
                    # CHECK IF WE ARE CLOSE
                    if maxSize > 400:
                        # keep track that we are seeing a close face
                        if not bCloseFace:
                            bCloseFace = True
                            timeSeenClose = time.time()

                        if time.time() - timeSeenClose > timeLookClose:
                            # switch to look at paper
                            robotBehavior = PAPER
                            bStarted = False
                            bCloseFace = False



                    # arm.set_position(pitch=dTilt, roll=dPan, relative=True, speed=500, mvacc=2000, wait=False)
                    

                    # DOESN'T WORK
                    # dollyX = offsetX
                    # dollyZ = offsetY
                    # ret = arm.set_position_aa(axis_angle_pose=[dollyX, 0, dollyZ, 0, dTilt, dPan], speed=700, relative=True, wait=False, radius=1.0)

                    # arm.set_tool_position(pitch=dTilt, wait=False)
                    # move j1
                    # code = arm.set_servo_angle(servo_id=1, angle=dPan, relative=True, is_radian=False, wait=True)
                    # print(code)
                    
                else:
                    if bStarted: 
                        bStarted = False
                        timeLastSeen = time.time()

                    if not bStarted and time.time()-timeLastSeen > 3.0:
                        # lost face, reset timer
                        startTime = time.time()

                        # move back to center
                        lookForward()
                        

                if time.time() - startTime > timeLookFace: 
                    robotBehavior = PAPER
                    bStarted = False

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

