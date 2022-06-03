# summer-robots
experimenting with the xarm7 in the summer! 2021, 2022

Exploring [ofxRobotArm](#ofxrobotarm), [ROS](#ros) and [python](#python).


# ofxRobotArm

## ofxRobotArm on Linux

__Download and install openFrameworks__

download from here: https://openframeworks.cc/download/

run the install scripts located in _scripts/linux/ubuntu_:

```
cd scripts/linux/ubuntu/
sudo ./install_dependencies.sh
sudo ./install_codecs.sh
```

compile the project generator located in _scripts/linux_:

```
cd ..
compilePG.sh
```

__2. Download ofxRobotArm and install to the oF/addons directory__

Install git-lfs before you clone ofxRobotArm:
```
sudo apt-get install git-lfs
```

change to the addons directory and clone ofxRobotArm:
```
git clone https://github.com/CreativeInquiry/ofxRobotArm
cd ofxRobotarm
./install_dependencies.sh
sudo apt-get install libboost-all-dev
```

run the project generator on the `ofxRobotArm/example-urdf` folder.

install the following (fixes cairo.h error…)
```
sudo apt-get install libyaml-cpp-dev
sudo apt-get install libnlopt-dev
```

make the example-urdf project:
```make```

# ROS

## ROS 1

# Python

## xArm-python-sdK
here is the UFACTORY xArm - https://github.com/xArm-Developer/xArm-Python-SDK

## our python code

# Reference
- UFACTORY Forums (great place to search for help): http://forum.ufactory.cc/
- xarm7 with pybullet: 
