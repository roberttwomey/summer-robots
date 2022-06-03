# summer-robots
experimenting with the xarm7 in the summer! 2021, 2022

Exploring [ofxRobotArm](#ofxrobotarm), [ROS](#ros) and [python](#python).


# ofxRobotArm

## ofxRobotArm on Linux

__Download and install openFrameworks__

Cownload from here: https://openframeworks.cc/download/

Run the install scripts located in _scripts/linux/ubuntu_:

```
cd scripts/linux/ubuntu/
sudo ./install_dependencies.sh
sudo ./install_codecs.sh
```

Compile the project generator located in _scripts/linux_:

```
cd ..
compilePG.sh
```

__2. Download ofxRobotArm and install to the _oF/addons_ directory__

Install git-lfs before you clone ofxRobotArm (because the repo has some large files included):
```
sudo apt-get install git-lfs
```

change to the _oF/addons_ directory and clone [ofxRobotArm](https://github.com/CreativeInquiry/ofxRobotArm):
```
git clone https://github.com/CreativeInquiry/ofxRobotArm
cd ofxRobotarm
./install_dependencies.sh
sudo apt-get install libboost-all-dev
```

__3. Compile and run the _example-urdf_

Run the project generator on the `ofxRobotArm/example-urdf` folder.

Install the following libraries on your ubuntu system (fixes the cairo.h error…)
```
sudo apt-get install libyaml-cpp-dev
sudo apt-get install libnlopt-dev
```

Make the example-urdf project:
```
cd addons/ofxRobotArm/example-urdf
make```

__3. Run the program__

To run the program, you can use the following command from the program folder:
```make RunRelease```

When you first run eample it will crash with a segmentation fault. This is because it is trying to load a urdf model file that is not present in the data folder: _example-urdf/bin/data/relaxed_ik_core/config/urdfs/irb4600_60_205.urdf_ folder. You will need to either switch to a different robot arm (for instance UR10) in [_ofApp.cpp_] in the example (see [this line here](https://github.com/CreativeInquiry/ofxRobotArm/blob/ef7bff0111d5271cac52c5c30a93675a658ed035/example-urdf/src/ofApp.cpp#L10)), or copy the correct file to the data path above. The file, irb4600_60_205.urdf is [here](https://github.com/CreativeInquiry/ofxRobotArm/blob/beta.1/data/relaxed_ik_core/config/urdfs/irb4600_60_205.urdf).

# ROS

## ROS 1

# Python

## xArm-python-sdK
here is the UFACTORY xArm - https://github.com/xArm-Developer/xArm-Python-SDK

## our python code

# Reference
- UFACTORY Forums (great place to search for help): http://forum.ufactory.cc/
- xarm7 with pybullet: 
