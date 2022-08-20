#!/usr/local/bin/python3

# Simple text programs for python control of xARM
# reference for https://github.com/xArm-Developer/xArm-Python-SDK

from xarm.wrapper import XArmAPI
import time
import sys
import cv2
import numpy as np

arm = XArmAPI('192.168.4.15')
arm.connect()
arm.set_pause_time(0.2)
time.sleep(3)


pauseBetween = True
simulate = False

arm.set_simulation_robot(on_off=simulate)

rest = [250, 0, 120, 180, 0, 0]

topleft = [222.7, -256.0, 110.1, 180, 0, 0]
topleftup = list(topleft)
topleftup[2]+=10
botleft = [557.2, -256.0, 110.1, 180.0, 0.0, 0.0]
botright = [563.1, 250.0, 110.1, 180.0, 0.0, 0.0]
topright = [240.7, 250.0, 110.1, 180.0, 0.0, 0.0]

# Create Transform
points1=np.array([[0.0,0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]])
points2=np.array([topleft[:2], botleft[:2], botright[:2], topright[:2]])

# reference https://www.pythonpool.com/cv2-findhomography/
H, status = cv2.findHomography(points1, points2)
print(H, status)

def calcTransformed(matrix, p):
	px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
	py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
	p_after = (int(px), int(py))
	return p_after

def calcArmPos(matrix, p):
	p_after = calcTransformed(matrix, p)
	position = [p_after[0], p_after[1], 110.1, 180.0, 0.0, 0.0]
	return position

# print(transformedPoints)
# exit()

# joint angle locations
lookdown = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
lookforward = [0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0]

# width = 609.6
# height = 300#457.2

thisspeed = 400 #200 #120 # 350

# positions = [
# 	rest,
# 	topleftup, 
# 	topleft, 
# 	botleft, 
# 	botright, 
# 	topright,
# 	topleft,
# 	topleftup,
# ]

# go to look
code = arm.set_servo_angle(angle=lookforward, speed=50, mvacc=500, wait=True, radius=-1.0)

arm.set_position(*rest, speed=thisspeed, wait=True)
arm.set_position(*topleftup, speed=thisspeed, wait=True)

points = [ (0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]

for point in points:
	position = calcArmPos(H, point)
	arm.set_position(*position, speed=thisspeed, wait=True)
	print("moving to %s" % position)

	print()
	if pauseBetween:
		input("Press Enter to continue...")

print("done.")
code = arm.set_servo_angle(angle=lookforward, speed=50, mvacc=500, wait=True, radius=-1.0)





# >>> from xarm.wrapper import XArmAPI
# >>> arm = XArmAPI('192.168.4.15')
# is_old_protocol: False
# version_number: 1.6.9
# >>> arm.connect()
# >>> code, position = arm.get_position()
# >>> position
# [206.0, -0.0, 120.5, 180.00002, -0.0, 0.0]
# >>> arm.set_position(position[0]+300, *position[1:])
# 0
# >>> arm.set_position(position[0]+400, *position[1:])
# 0
# >>> arm.set_position(position[0]+500, *position[1:])
# 0
# >>> arm.set_position(position[0]+400, *position[1:])
# 0
# >>> arm.set_position(position[0]+400, position[1]+200, *position[2:])
# 0
# >>> arm.set_position(position[0]+400, position[1]+300, *position[2:])
# 0
# >>> arm.set_position(position[0], position[1]+300, *position[2:])
# 0
# >>> arm.set_position(position[0], position[1]-300, *position[2:])
# 0
# >>> arm.set_position(position[0]+400, position[1]-300, *position[2:])
# 0
# >>> arm.set_position(position[0]+400, position[1], *position[2:])
# 0
# >>> arm.move_gohome()
# 0
# >>> exit()