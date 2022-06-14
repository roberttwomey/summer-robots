"""
Convert svg line segments file to json
rtwomey@unl.edu | 2022

"""
import sys
import json
from svgpathtools import svg2paths

# read svg
# paths, attributes = svg2paths('../data/svg/CLIPassoJasperMarker_segs.svg')

if len(sys.argv) > 1:
    infile = sys.argv[1]
    jsonfile = sys.argv[1].split(".svg")[0]+".json"

paths, attributes = svg2paths(infile)

# print(paths)
print(len(paths))

out_paths = []

for path in paths:
    this_path = []
    for seg in path: 
        for point in seg:
            point = { "x":point.real, "y":point.imag}
            # print(point)
            this_path.append(point)
    out_paths.append(this_path)

# print(out_paths)

print("writing {}".format(jsonfile))
with open(jsonfile, 'w') as outfile:
    json.dump(out_paths, outfile, indent=4)