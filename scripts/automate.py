#!/usr/bin/env python3

import rospy
import os
import rospkg

for i in range(1):
    dirname = rospkg.RosPack().get_path('swarm_aggregation')
    os.system(f"rm -rf {dirname}/Data/Threshold_0.7/Data{i}/*")
    os.system(f"rosparam set /iteration/ {i}")
    os.system("roslaunch swarm_aggregation nine.launch")