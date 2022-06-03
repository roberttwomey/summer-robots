# summer-robots
experimenting with xarm7 summer ~2021~ 2022 with

# ofxRobotArm

## ofxRobotArm on Linux

install openframeworks. download from here: 
https://openframeworks.cc/download/

```
cd scripts/linux/ubuntu/
sudo ./install_dependencies.sh
sudo ./install_codecs.sh
cd ..
compilePG.sh
```

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
