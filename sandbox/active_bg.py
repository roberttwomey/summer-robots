#!/usr/bin/env python3
#
# Simple last minute background subtraction
# from here https://stackoverflow.com/questions/34615331/opencv-background-subtraction

#
import sys
import math
import time
import datetime
import random
import traceback
import threading

import cv2

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


pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.4.15')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

variables = {}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

# downAngle = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
# downAngle = [-0.2, -0.4, 0, 92.7, 1.1, 81.1 -0.2] # v1
# downAngle = [0, 1.8, 0, 100, 0.1, 96.7, 0] # v2
downAngle = [0.8, -0.9, 0.8, 78.8, 0.1, 78.3, 1.5]
downPos = [467.8, 7, 569.1, -179.9, -1.5, 0]


CAMERA_NUM = 0 # mac mini
# CAMERA_NUM = 1 # laptop

# To capture video from webcam. 
cap = cv2.VideoCapture(CAMERA_NUM)

capWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
capHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

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

# Rotation
if not params['quit']:
    params['angle_acc'] = 1145
if not params['quit']:
    params['angle_speed'] = 80
    # if params['quit']:
    
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=downAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        # code = arm.set_servo_angle(angle=[0.6, -22.5, -0.9, 81.2, 1.8, 100.8, 0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))

# arm.set_position(pitch=-10.0, relative=True, wait=True)
# arm.set_position(pitch=-20.0, relative=True, wait=True)
# arm.set_position(pitch=-10.0, relative=True, wait=True)
# arm.set_position(roll=-10.0, relative=True, wait=True)
# arm.set_position(roll=20.0, relative=True, wait=True)
# arm.set_position(roll=-10, relative=True, wait=True)

# back to forward
# arm.set_servo_angle(angle=[0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

time.sleep(3.0)

bg = cv2.imread("../background.jpg")

# Save the first image as reference
bg_gray = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)
bg_gray = cv2.GaussianBlur(bg_gray, (21, 21), 0)


while True:
        
    # Read the frame
    _, img = cap.read()
            
    if img is not None:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # In each iteration, calculate absolute difference between current frame and reference frame
        difference = cv2.absdiff(gray, bg_gray)

        # Apply thresholding to eliminate noise
        thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)

        cv2.imshow("thresh", thresh)
        key = cv2.waitKey(1) & 0xFF

    # if the `q` key is pressed, break from the lop
    if key == ord("q"):
        break

# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)

# free up camera
# Release the VideoCapture object
cap.release()
cv2.destroyAllWindows()
