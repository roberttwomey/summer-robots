#!/usr/local/bin/python3
from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.4.15')
arm.connect()
time.sleep(3)

arm.move_gohome()
code, position = arm.get_position()

pause = 10

arm.set_position(position[0], position[1]-300, *position[2:])
time.sleep(5)
arm.set_position(position[0]+400, position[1]-300, *position[2:])
time.sleep(5)
arm.set_position(position[0]+400, position[1]+300, *position[2:])
time.sleep(5)
arm.set_position(position[0], position[1]+300, *position[2:])
time.sleep(5)
arm.move_gohome()


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