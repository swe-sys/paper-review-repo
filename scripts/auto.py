#!/usr/bin/env python3
import os,glob,sys
import rospy

for i in range(5):
    sys.system("./sim.sh")
    #check for convergence
    #kill all 
