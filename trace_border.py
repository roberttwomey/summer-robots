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

# arm.move_gohome()
# code, home = arm.get_position()

pauseBetween = False

rest = [250, 0, 120, 180, 0, 0]
# forward = [223.9, 0, 400.0, 0.0, -90.0, 180.0]
# start = [230, 0, 110, 180, 0, 0]

topleftup = [231.4, -256.7, 120.0, 180.0, 0.0, 0.0]
topleft = [231.4, -256.7, 110.1, 180, 0, 0]
botleft = [557.2, -256.4, 110.1, 180.0, 0.0, 0.0]
botright = [563.1, 250.5, 110.1, 180.0, 0.0, 0.0]
topright = [240.7, 242.9, 110.1, 180.0, 0.0, 0.0]

# joint angle locations
lookdown = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
lookforward = [0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0]

# width = 609.6
# height = 300#457.2

speed = 300 #200 #120 # 350

# Test 1: SQUARE TEST
# positions = [
#  	rest,
# 	start,
# 	# [start[0]+400, *start[1:]],
# 	# start,
# 	# [start[0], start[1]-width/2, *start[2:]],
# 	# [start[0]+height, start[1]-width/2, start[2], *start[3:]],
# 	# [start[0]+height, start[1]+width/2, start[2], *start[3:]],
# 	# [start[0], start[1]+width/2, *start[2:]],
# 	start, 
# 	rest
# ]

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
	arm.set_position(*position, speed=speed, wait=True)#350)
	# arm.set_position(*position, speed=speed, radius=0, wait=True)#350)
	# arm.set_position(*position, speed=speed, radius=0, wait=False)#350)
	# arm.set_position(*position, speed=speed, wait=True)#350)

	print("moving to %s" % position)

	# while arm.get_is_moving():
	# 	print(".", end="")
	# 	sys.stdout.flush()
	# 	time.sleep(0.1)
	print()
	if pauseBetween:
		input("Press Enter to continue...")

print("done.")
code = arm.set_servo_angle(angle=lookforward, speed=20, mvacc=500, wait=True, radius=-1.0)
# code = arm.set_servo_angle(angle=lookdown, speed=20, mvacc=500, wait=True, radius=-1.0)
# code = arm.set_servo_angle(angle=[0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0], speed=20, mvacc=500, wait=True, radius=-1.0)

# pause = 10

# arm.set_position(position[0], position[1]-300, *position[2:])
# time.sleep(5)
# arm.set_position(position[0]+400, position[1]-300, *position[2:])
# time.sleep(5)
# arm.set_position(position[0]+400, position[1]+300, *position[2:])
# time.sleep(5)
# arm.set_position(position[0], position[1]+300, *position[2:])
# time.sleep(5)
# arm.move_gohome()


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