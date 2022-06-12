#!/usr/local/bin/python3

# Simple text programs for python control of xARM
# reference for https://github.com/xArm-Developer/xArm-Python-SDK

from xarm.wrapper import XArmAPI
import time
import sys

arm = XArmAPI('192.168.4.15')
arm.connect()
arm.set_pause_time(0.2)
time.sleep(3)

pauseBetween = False

rest = [250, 0, 120, 180, 0, 0]

topleft = [222.7, -256.0, 110.1, 180, 0, 0]
topleftup = list(topleft)
topleftup[2]+=10
botleft = [557.2, -256.0, 110.1, 180.0, 0.0, 0.0]
botright = [563.1, 250.0, 110.1, 180.0, 0.0, 0.0]
topright = [240.7, 250.0, 110.1, 180.0, 0.0, 0.0]

# joint angle locations
lookdown = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
lookforward = [0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0]

# width = 609.6
# height = 300#457.2

thisspeed = 400 #200 #120 # 350

positions = [
	rest,
	topleftup, 
	topleft, 
	botleft, 
	botright, 
	topright,
	topleft,
	topleftup,
]

# go to look
code = arm.set_servo_angle(angle=lookforward, speed=20, mvacc=500, wait=True, radius=-1.0)

for position in positions:
	arm.set_position(*position, speed=thisspeed, wait=True)
	# arm.set_position(*position, speed=speed, radius=0, wait=True)#350)
	# arm.set_position(*position, speed=speed, radius=0, wait=False)#350)
	# arm.set_position(*position, speed=speed, wait=True)#350)

	print("moving to %s" % position)

	print()
	if pauseBetween:
		input("Press Enter to continue...")

print("done.")
code = arm.set_servo_angle(angle=lookforward, speed=20, mvacc=500, wait=True, radius=-1.0)





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