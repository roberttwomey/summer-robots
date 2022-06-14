#!/Library/Frameworks/Python.framework/Versions/2.7/bin/python
"""
Convert JSON result file to plain text

rtwomey@uw.edu

2013-11-23

"""

from subprocess import call
import os
import sys
import simplejson
import numpy as np
import argparse


# polygon loading / sorting

def readJSON(filename):
    with open (filename, "r") as myfile:
        data = myfile.read()

    pydata = simplejson.loads(data)
    print(len(pydata))
    polys = []

    for path in pydata:
        # print(pydata[path])
        thispath = []

        for point in path:
            print(point)
            x = point['x']
            y = point['y']
            print(x,y)
            thispath.append((x, y))
        
        if(len(thispath) > 5):
            polys.append(thispath)
    
    # exit()
        # thispath = []
        # for point in path:
        #     # thispath.append([0, point["x"]/640.0, point["y"]/480.0, 1.0])
        #     thispath.append([point["x"], point["y"]])
        # # print(thispath)
        # polys.append(thispath)
        # # for point in path:
        # #     print(point)
        # # print("\n\n=================\n\n")
    return polys


# coordinate generation / sorting

def generateCoords(polys, minlen):
    """
    Generates list of x,y,z coordinates from points in json file.
    list of polys and coordinates.
    """
    
    wcount = 0
    scount = 0
    last = [0., 0.]
    lastz = 0. # pen up
    
    polycount = 0

    cbuff = []
    print(len(polys))
    for poly in polys:

        first = True
        
        for i, coord in enumerate(poly):            
            x = coord[0]
            y = coord[1]

            # check distance threshold
            if np.linalg.norm(np.array(coord)-np.array(last)) > minlen:               
                if first:
                    # move to first point, pen up
                    cbuff.append([x, y, 0.])
                    last = coord
                    first = False
                    wcount = wcount + 1

                # write to file, pen down
                cbuff.append([x, y, 1.])                                
                last = coord
                wcount = wcount + 1

            else:
                # skipping
                scount = scount + 1

        # last
        # stay at last point, add pen up
        cbuff.append([last[0], last[1], 0.])
        wcount = wcount + 1                 

        polycount = polycount + 1

    print("{0} points buffered out of {1} from {2} polygons.".format(wcount, scount, polycount))

    return cbuff



def sortPolys(polys):
    """
    Starting at 0,0, sorts all polys in array by nearest neighor,
    flipping the beginning and ends if necessary.

    Returns sorted poly array.
    """
    
    print("starting sort...")

    print("(",len(polys),")")
    
    sortedpolys = []
    last = [0., 0.]

    used = [ False ] * len(polys)    
    done = False
    count = 0

    #print used
    while not done:

        minlen = 99999
        closest = -1
        flipped = None
        
        for i, poly in enumerate(polys):
            #print i, used[i]
            
            if not used[i]:
                begin = poly[0] # first coordinate
                end = poly[-1] # last coordinate

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

                print("  testing {0}, len1: {1:.2f}\tlen2: {2:.2f}\tmin: {3:.2f}".format(i, beginlen, endlen, minlen))

        # we have our victor
        print("best {} flipped {} {:.2f}".format(closest, flipped, minlen))
        if flipped:
            sortedpolys.append(polys[closest][::-1])
        else:
            sortedpolys.append(polys[closest])

        used[closest] = True
        last = sortedpolys[-1][-1]
        count = count + 1
        sys.stdout.flush()
        
        if count == len(polys):
            done = True

    print("done. sorted {0} polys".format(count))
    
    return sortedpolys


def removeDuplicates(coords):

    if len(coords) < 2:
        return coords

    startlen = len(coords)
    
    last = coords[0]

    cleaned = []

    cleaned.append(coords[0])

    for curr in coords[1:]:
        if curr[2] < 1. and last[2] < 1.:
            continue

        cleaned.append(curr)
        last = curr

    print("{0} left from {1} after removing duplicate fast moves".format(len(cleaned), startlen))

    return cleaned


def removeShort(coords):

    if len(coords) < 3:
        return coords

    startlen = len(coords)
    #print startlen, "coords"
    cleaned = []
    cleaned.append(coords[0])
    
    for i in range(len(coords)-2):
        
        lastc = coords[i]
        currc = coords[i+1]
        nextc = coords[i+2]

        #print i, lastc, currc, nextc

        if currc[2] < 1.:
            if (lastc[2] < 1.) and (nextc[2] < 1.):
                print(i+1,"*delete*", lastc[2], nextc[2])
        else:
            #print i+1, lastc[2], nextc[2]
            cleaned.append(currc)

    print("{0} left from {1} after removing short paths".format(len(cleaned), startlen))

    return cleaned


def writeCoords(filename, coords):

    outf = open(filename, "w")
    wcount = 0
    for i, coord in enumerate(coords):
       outf.write("{0:.5f}\t{1:0.5f}\t{2:0.5f}\t{3:0.1f}\n".format(coord[0], coord[1], coord[2], -1.0))
       wcount = wcount + 1
       
    outf.close()

    print("{0} coords written.".format(wcount))


def main():

    # handle command line arguments
    parser = argparse.ArgumentParser(description='Convert POTRACE json file to plain text (argparse library is required)')
    parser.add_argument('infile',
            help='output of potrace command (.json)')
    parser.add_argument("outfile", help="destination text file (.txt)")
    parser.add_argument('minlen', help='minimum segment length', type=float,
                        default=0.05)
    parser.add_argument('height', type=float)
    
    args = parser.parse_args()
    infile = args.infile
    outfile = args.outfile
    minlen = args.minlen
    height = args.height
    
    print("=== LOAD VECTORIZED IMAGE ===")
    polys = readJSON(infile)

    print("=== NEAREST NEIGHBOR SORT ===")
    sortedpolys = sortPolys(polys)
                       
    print("=== FILTER COORDS ===")
    # coords = generateCoords(polys, minlen)
    coords = generateCoords(sortedpolys, minlen)
    # cleaned = removeDuplicates(coords)


    print("=== WRITE COORDS TO TEXT ===")
    # writeCoords(outfile, cleaned) #  short) # 
    writeCoords(outfile, coords) #  short) # 
    
if __name__ == "__main__":
    main()
