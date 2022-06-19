# summer-robots

Experimenting with the xarm7 in the summer! 2021, 2022

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

__Download ofxRobotArm and install to the _oF/addons_ directory__

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

__Compile the example program__

edit the `config.make` file in `example-urdf` - remove/comment out the OF_ROOT line (or add the path to your openFrameworks root directory)

Run the project generator on the `ofxRobotArm/example-urdf` folder.

Install the following libraries on your ubuntu system (fixes the cairo.h error…)
```
sudo apt-get install libyaml-cpp-dev
sudo apt-get install libnlopt-dev
```

Make the example-urdf project:
```
cd addons/ofxRobotArm/example-urdf
make
```

__Run the example program__

To run the program, you can use the following command from the program folder:
```make RunRelease```

When you first run eample it will crash with a segmentation fault. This is because it is trying to load a urdf model file that is not present in the data folder: _example-urdf/bin/data/relaxed_ik_core/config/urdfs/irb4600_60_205.urdf_. You will need to either switch to a different robot arm (for instance UR10) in _ofApp.cpp_ in the example (see [this line here](https://github.com/CreativeInquiry/ofxRobotArm/blob/ef7bff0111d5271cac52c5c30a93675a658ed035/example-urdf/src/ofApp.cpp#L10)), or copy the correct URDF file to the data path above. 

The file, irb4600_60_205.urdf is [here](https://github.com/CreativeInquiry/ofxRobotArm/blob/beta.1/data/relaxed_ik_core/config/urdfs/irb4600_60_205.urdf).

# ROS

## ROS 1

# Python

## xArm-python-sdK
here is the UFACTORY xArm - https://github.com/xArm-Developer/xArm-Python-SDK

## our python code

## pybullet

<img width="400" alt="image" src="https://user-images.githubusercontent.com/1598545/171915352-566fb972-045a-4b04-8016-76c592f33d28.png">

[a thread from ufactory forums about this](https://forum.ufactory.cc/t/smoother-linear-motion-with-xarm-7-leap-motion/2285/9)


Create a mamba environment: 
```mamba create -n pybullet python=3.9```

Open this environment
```conda activate pybullet```

Install numpy: 
```mamba install numpy```

Install pybullet:
```pip3 install pybullet```

Clone xArm-Python-SDK:
```
git clone https://github.com/erwincoumans/xArm-Python-SDK
```

Run the example (which traces a cricle in 3d modeled environment): 
```
cd example/wrapper/xarm7
python3 loadxarm_sim.py
```

To run this on the real arm: 
```python3 xarm_real_ik.py```

# Reference
- UFACTORY Forums (great place to ask for help): http://forum.ufactory.cc/
