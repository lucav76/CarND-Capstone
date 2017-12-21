#!/bin/bash

#build packages
catkin_make

#source the setup file
source devel/setup.bash

#launch the package for running in the simulator
roslaunch launch/styx.launch
