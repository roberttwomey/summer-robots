#!/usr/local/bin/python3

# Simple text programs for python control of xARM
# this one reads in a series of pen recorder coordinates and 
# replays them on the xarm

# reference for https://github.com/xArm-Developer/xArm-Python-SDK

from xarm.wrapper import XArmAPI
import time
import sys
import numpy as np
import argparse
import cv2


def readDrawingData_rk(filename, width, height):

    f = open(filename)

    lines = [line.rstrip('\n') for line in f]
    points = []

    print("read {0} lines from file".format(len(lines)))

    # min/max state
    minx = 0
    miny = 0
    bFirstPoint = True
    minlen = 5.0#15.0#


    lastpos = None

    for i, l in enumerate(lines):
        # print l
        fields = np.array(l.split('\t')).astype(np.float)
        
        if len(fields)<4:
            print("too many columns in file: %s" % i)
            exit()

        if len(fields) == 6:
            # six fields including tilt
            seconds, xpos, ypos, zpos, tiltx, tilty = fields
        else:
            seconds, xpos, ypos, zpos = fields

        xpos = xpos * width#1.29237
        ypos = ypos * height

        pos = np.array([xpos, ypos])

        if bFirstPoint:
            bFirstPoint = False
            lastpos = pos
            points.append([seconds, xpos, ypos, zpos > 0])
            continue
        else: 
            if np.linalg.norm(pos-lastpos) < minlen:
                continue
            else: 
                lastpos = pos
                points.append([seconds, xpos, ypos, zpos > 0])
            

        #print "time",millis,
        

    print("done reading file.")

    f.close()

    # find bounding box for penstrokes
    nppoints = np.array(points)
    mins = np.min(nppoints, axis=0)
    maxes = np.max(nppoints, axis=0)

    minx, miny = mins[1:3]
    maxx, maxy = maxes[1:3]

    return nppoints, (minx, maxx, miny, maxy)


def mapToScreen(point, minx, maxx, miny, maxy, outwidth):

    width = maxx - minx
    height = maxy - miny

    # check whether we are in portrait or landscape
    if width >= height:
        scale = outwidth / float(width)
        margin = width * 0.0
        dominant = width
        
    else:
        scale = outwidth / float(height)
        margin = width * 0.0
        dominant = height

    xoff = 0 - margin - ((dominant - width) / 2.0)
    yoff = 0 - margin - ((dominant - height) / 2.0)
    
    offset = np.array((-xoff, -yoff)) * scale

    # print(scale, offset)
    mappedpoint = (point * scale + offset)

    return mappedpoint

def filterToRobot(points, start_pos, pen_height, width, height, outwidth):
    outpoints = []
    for point in points:
        # print(point)
        xy = point[1:3]
        curr = mapToScreen(xy, 0, width, 0, height, outwidth*25.4)
                    
        # set radius > 0 for continuous motion
        outpoint = [ start_pos[0]+curr[1], 
                        start_pos[1]+curr[0],
                        start_pos[2]-(float(point[3])*pen_height),
                        *start_pos[3:], 15 ]

        # outpoint = [ start_pos[0]+curr[1], 
        #                 start_pos[1]+curr[0],
        #                 start_pos[2]-(float(point[3])*pen_height),
        #                 *start_pos[3:]]

        outpoints.append(outpoint)
    # print(outpoints)
    # exit()
    return outpoints



def main():
    
    # Define command line argument interface
    parser = argparse.ArgumentParser(description='Stream a pen time series to the xArm7')
    parser.add_argument('width', type=float, default=640)
    parser.add_argument('height', type=float, default = 480)
    parser.add_argument('outwidth', type=float, default=12.0, help='width of output in inches')
    parser.add_argument('files', nargs='*', help='glob of input files')
    parser.add_argument('-v','--verbose',action='store_true', default=False, 
            help='verbose coordinate output text')
    parser.add_argument('-t', '--test', action='store_true', default=False, 
        help="don't open serial")
    parser.add_argument('-p', '--png', action='store_true', default=False,
        help="write png image of output")
    
    # handle arguments
    args = parser.parse_args()

    width = args.width
    height = args.height
    outwidth = args.outwidth
    infiles = args.files

    verbose = False
    if args.verbose: verbose = True

    doTest = False
    if args.test: doTest = True

    doPng = False
    if args.png: doPng = True


    # read in input JSON files
    paths = []

    print("there are {} files...".format(len(infiles)))

    # load coordinates
    for infname in infiles:
        
        print("reading {}...".format(infname), end="")
        # read in input file (drawing reording)
        points, boundingbox = readDrawingData_rk(infname, width, height)
        
        paths.append(points)
        # for path in points:
        #     paths.append(path)

    # print(len(paths))
    # print(paths)
    # print(boundingbox)
    # print(len(paths[0]))
    # exit()


    # set up drawing
    if doPng:
        # line params
        linewidth = 3
        WHITE = (255, 255, 255)
        BLUE = (255, 0, 0)
        CYAN = (255, 255, 0)
        BLACK = (0, 0, 0)

        pixheight = int(height * 4)
        pixwidth = int(width * 4)
        
        # blank image 
        img = np.zeros((pixheight, pixwidth, 3), np.uint8)
        cv2.rectangle(img, (0,0), (pixwidth, pixheight), WHITE, cv2.FILLED)


    # main loop
    z = 0
    lastz = 0
    count = 0
    lastpoint = None

    # where to start (for resuming)
    firstfile = 0
    firsttrail = 0
    firstpoint = 0


    start_time = time.time()

    fileidx=firstfile
    count = 0

    # robot params
    xarm_rest_pos = [220, 0, 120.5, 180, 0, 0]
    xarm_start_pos = [220, 0-(outwidth*0.5*25.4), 120.5, 180, 0, 0]
    xarm_speed = 350
    # xarm_mvacc = 800
    xarm_height = 15.0 #mm
    # xarm_speed1=1000


        # connect to drawing machine
    if not doTest:
        print("Initializing xArm...")
        arm = XArmAPI('192.168.4.15')
        arm.connect()

        # from http://download.ufactory.cc/xArm_Python_SDK/1001-xArm-linear%20motion-example1.py
        # arm.motion_enable(enable=True)
        # arm.set_mode(0)
        # arm.set_state(state=0)
        # arm.set_tcp_jerk(1000)
        # arm.set_joint_jerk(20,is_radian=True)
        
        # arm.set_pause_time(1)
        # arm.save_conf()

        # arm.reset(wait=True)

        arm.set_position(*xarm_rest_pos, speed=350)
        print("moved to rest_position")

    new_paths = filterToRobot(paths[0], xarm_start_pos, xarm_height, width, height, outwidth)


    try:
        n = 500#25#100
        for i in range(0, len(new_paths), n):
            points = new_paths[i:i +n]
            
            # positions = [ [xarm_start_pos[0]+curr[1]*outwidth*25.4, 
            #             xarm_start_pos[1]+curr[0]*outwidth*25.4,
            #             xarm_start_pos[2]-(float(z)*xarm_height),
            #             *xarm_start_pos[3:] ] for curr in points]

            try: 
                # print(points)
                count += n

                arm.move_arc_lines(points, speed=50, mvacc=2000, times=1, wait=True)

        # for path in paths[firstfile:]:
        #     # print("starting file {}".format(fileidx))
        #     fileidx += 1
        #     pathidx = 0

            
        #     if verbose:
        #         print("starting file {}:path {}".format(fileidx, pathidx))
        #     pathidx += 1
        #     points = path[firstpoint:]

        #     # move to initial point with penup
        #     first_point = True
        #     pointidx = 0
        #     for point in points:
        #         try:
        #             xy = point[1:3]
        #             z = point[3]
        #         except:
        #             print("problem", point)
        #             # print(len(point)) 
        #             print(count)


        #         try:
        #             # iterate line counter and check length
        #             count += 1 
        #             pointidx += 1

        #             curr = mapToScreen(xy, 0, width, 0, height, outwidth*25.4)
                    
        #             position = [
        #                 xarm_start_pos[0]+curr[1], 
        #                 xarm_start_pos[1]+curr[0],
        #                 xarm_start_pos[2]-(float(z)*xarm_height),
        #                 *xarm_start_pos[3:]
        #                 ]
        #             # print(position)
                        
        #             if not doTest and (z > 0 or lastz > 0) :
        #                 # arm.set_position(*position, speed=xarm_speed, mvacc=xarm_mvacc, radius=10)
        #                 arm.set_position(*position, speed=xarm_speed, radius = 3)#, radius=100, wait=False)

        #                 # while arm.get_is_moving():
        #                 #     print(".", end="")
        #                 #     sys.stdout.flush()
        #                 #     time.sleep(0.1)
        #                 # print()

        #             if doPng:

        #                 currpix = mapToScreen(xy, 0, width, 0, height, pixwidth)
                        
        #                 # print(lastpix, currpix)
        #                 color = BLUE
        #                 if lastz == 0:
        #                     color = CYAN
        #                 if not first_point:
        #                     cv2.line(img, (np.float32(lastpix[0]), np.float32(lastpix[1])), (np.float32(currpix[0]), np.float32(currpix[1])), (color), linewidth, cv2.LINE_AA)

        #                 lastpix = currpix
        #                 lastz = z

                # if first_point:
                #     first_point = False


                # if count % 500 == 0:
                #     print(".", end="")
                if count % 1000 == 0:
                    # print("\n{}:{}:{}\t{}".format(fileidx, pathidx, pointidx, curr), end="")
                    print("{}".format(count))

                # if verbose:
                #     print("{}:\t{}".format(count, curr))

                sys.stdout.flush()


            except (KeyboardInterrupt):
                nb = None
                while nb is not "":
                    nb = input("\ncommand (? for help):")
                    if nb == "q":
                        print("quitting at point:")
                        print("{}:{}:{}\t{}".format(fileidx, pathidx, pointidx, curr), end="")
                        if not doTest:
                            arm.set_position(*xarm_rest_pos)
                        raise(KeyboardInterrupt)
                        break
                    elif nb == "h":
                        if not doTest: ad.moveto(0,0)
                    elif nb == "d":
                        if not doTest: ad.pendown()
                    elif nb == "f":
                        if not doTest: ad.disable()
                    elif nb == "u":
                        if not doTest: ad.penup()
                    elif nb == "?":
                        print("""h to home
    d to put pen down
    u to lift pen up
    f to move carriage freely (disable motors)
    q to go home and quit
    type a sentence to pass as gcode command
    """),
                    else:
                        if not doTest: ad.write(nb) # send to grbl

    except (KeyboardInterrupt):
        print("exiting...")

        pass

    # close drawing machine
    if not doTest: 
        arm.set_position(*xarm_rest_pos)

    print("\n")

    # write out image
    if doPng:
        outfname='output.png'
        print("Saving image {0}".format(outfname))
        cv2.imwrite(outfname, img)

    print("Program read {0} points in {1} files in {2:0.2f} seconds".format(count, fileidx, time.time() - start_time))


if __name__ == "__main__":
    main()
