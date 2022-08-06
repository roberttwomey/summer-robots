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

liftHeight = 10.0#15.0
liftSpeed = 500
drawSpeed = 50
#drawSpeed = 75# still jittery on circles
drawRadius = 0.0
drawAccel = 500
pathChunkLength = 25#3#500#20 #10#25#100
dwellTime = 0.0 #0.1 default

debug = False

# arm parameters
variables = {}
# params = {'speed': 400, 'acc': 2000, 'angle_speed': 500, 'angle_acc': 5000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
# params = {'speed': 100, 'acc': 500, 'angle_speed': 500, 'angle_acc': 5000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
params = {'speed': drawSpeed, 'acc': drawAccel, 'angle_speed': 500, 'angle_acc': 5000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

frontAngle = [0, 2.5, 0, 37.3, 0, -57.3, 0]

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
    position = [p_after[0], p_after[1], zheight, 180.0, 0.0, 0.0] # no radius
    # position = [p_after[0], p_after[1], zheight, 180.0, 0.0, 0.0, 1.0] # radio, I plan motion (?)
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

# ==== Wrangle pen paths ====

# recpoints, bounds = readDrawingRecording("data/3891_0911092146.txt")
# unsorted_paths = readJSON("../data/CLIPassoGAN_sample02_segs.json")
# unsorted_paths = readGeoJSON("../data/GAN_sample02.json")

# unsorted_paths = readJSON("../data/sample01_ink_flatten.json", 908, 681)
# paths = sortPaths(unsorted_paths)
# renderAsImage(paths, "../data/sample01_ink_flatten.png", 1024, 1024)


if len(sys.argv) > 1:
    infile = sys.argv[1]
else: 
    infile = "../data/sample01.json"

unsorted_paths = readJSON(infile, 908, 600)#681)
paths = sortPaths(unsorted_paths)

pngfile = infile.split(".json")[0]+"_paths.png"
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
arm.set_simulation_robot(on_off=simulate)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_pause_time(dwellTime)


# Register error/warn changed callback

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

        
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)

def lookForward():
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=frontAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))

# Position Arm

# joint angle locations
lookdown = [0.4, 5.6, -1.0, 110.1, 2.3, 101.6, -0.1]
lookforward = [0.0, -45.0, 0.0, 0.0, 0.0, -45.0, 0.0]


# go to look
lookForward()

arm.set_position(*rest, speed=liftSpeed, wait=True)
# arm.set_position(*topleftup, speed=params['speed'], wait=True)


# ==== DO ACTUAL DRAWING ====

try:
    for path in new_paths:

        n = pathChunkLength
        count = 0
        
        # move to start with pen in air
        start_point = list(path[0])
        start_point[2] += liftHeight
        print("path started: {}".format(start_point))
        # arm.set_position(*start_point, speed=params['speed'], wait=True)
        # arm.set_position(*start_point, speed=drawSpeed, mvacc=drawAccel, wait=False, radius=drawRadius)
        arm.set_position(*start_point, speed=drawSpeed, mvacc=drawAccel, wait=False)

        for point in path:
            # position = calcArmPos(H, point)
            # arm.set_position(*point, speed=drawSpeed, wait=True)
            # arm.set_position(*point, speed=drawSpeed, wait=False, radius=0.1)
            # arm.set_position(*point, wait=False, radius=0.1)
            # arm.set_position(*point, wait=False, radius=None) # MoveLine, linear motion
            arm.set_position(*point, wait=False, mvacc=drawAccel, radius=drawRadius) # MoveArcLine with interpolation, linear arc motion

        # for i in range(0, len(path), n):
        #     streampoints = path[i:i+n]

        #     print("drawing points {}:{} out of {}...".format(i, i+n-1, len(path)))
        #     sys.stdout.flush()

        #     code = arm.move_arc_lines(streampoints, speed=drawSpeed, mvacc=drawAccel, times=1, wait=True)
        #     time.sleep(dwellTime)

            # code = arm.move_arc_lines(streampoints, speed=params['speed'], mvacc=params['acc'], times=1, wait=True)
            # if code != 0:
            #     # release error
            #     # arm.release_error_warn_changed_callback(error_warn_change_callback)
            #     print("Releasing error and going home")
            #     arm.clean_warn()
            #     arm.clean_error()
            #     arm.motion_enable(True)
            #     arm.move_gohome(wait=True)
            #     lookForward()

        # lift pen in air before starting next path
        end_point = list(path[-1])
        end_point[2] += liftHeight
        # arm.set_position(*end_point, speed=params['speed'], wait=True)
        # arm.set_position(*end_point, speed=liftSpeed, mvacc=drawAccel, wait=True, radius=drawRadius)
        arm.set_position(*end_point, speed=drawSpeed, mvacc=drawAccel, wait=True)

        print("path ended, sent {} points".format(len(path)))#: {}".format(end_point))
        sys.stdout.flush()

        # time.sleep(dwellTime)
        print("done")
        sys.stdout.flush()        

        while arm.get_is_moving():
            print(".", end="")
            sys.stdout.flush()
            time.sleep(0.01)



    print("done.")
except KeyboardInterrupt:
    print("quitting.")

sys.stdout.flush()
code = arm.set_servo_angle(angle=lookforward, speed=params['speed'], mvacc=params['acc'], wait=True, radius=-1.0)
