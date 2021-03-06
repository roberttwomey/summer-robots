#!/usr/local/bin/python3

# Simple text programs for python control of xARM
# reference for https://github.com/xArm-Developer/xArm-Python-SDK

from xarm.wrapper import XArmAPI
import time
import sys

arm = XArmAPI('192.168.4.15')
arm.connect()

# turn off simulation
arm.set_simulation_robot(on_off=False)

time.sleep(3)

# arm.move_gohome()
# code, home = arm.get_position()
start = [220, 0, 120.5, 180, 0, 0]

positions = [
	[*start],
	[start[0], start[1]-300, *start[2:]],
	[start[0]+400, start[1]-300, *start[2:]],
	[start[0]+400, start[1]+300, *start[2:]],
	[start[0], start[1]+300, *start[2:]],
	[*start]
]

for position in positions:
	arm.set_position(*position, speed=350)
	print("moving to %s" % position)

	while arm.get_is_moving():
		print(".", end="")
		sys.stdout.flush()
		time.sleep(0.1)
	print()

print("done.")
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