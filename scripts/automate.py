#!/usr/bin/env python3
import rospy
import os

for i in range(10):
    os.system("rosparam set /iteration/ {i}")
    os.system("roslaunch swarm_aggregation hexagon.launch")