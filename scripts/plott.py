#!/usr/bin/env python3
from __future__ import division
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from swarm_aggregation.msg import botPose, bot
from math import cos, sin, pi
import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt

plt.ion()
class yolo():
    def __init__(self):
        self.odom = {}
        self.goal = {}
        self.bot_id = []
        rospy.Subscriber('/obs_data', botPose, self.pose_listener)
        # rospy.Subscriber('/obs_data', botPose, self.store_data)
        rospy.Subscriber('/tb3_0/goal',Point,self.goal_listener,'/tb3_0/')
        rospy.Subscriber('/tb3_1/goal',Point,self.goal_listener,'/tb3_1/')
        rospy.Subscriber('/tb3_2/goal',Point,self.goal_listener,'/tb3_2/')
        rospy.Subscriber('/tb3_3/goal',Point,self.goal_listener,'/tb3_3/')
        rospy.Subscriber('/tb3_4/goal',Point,self.goal_listener,'/tb3_4/')
        rospy.Subscriber('/tb3_5/goal',Point,self.goal_listener,'/tb3_5/')
        rospy.Subscriber('/tb3_6/goal',Point,self.goal_listener,'/tb3_6/')
        rospy.Subscriber('/tb3_7/goal',Point,self.goal_listener,'/tb3_7/')
        rospy.Subscriber('/tb3_8/goal',Point,self.goal_listener,'/tb3_8/')
        rospy.Subscriber('/tb3_9/goal',Point,self.goal_listener,'/tb3_9/')
        rospy.Subscriber('/tb3_10/goal',Point,self.goal_listener,'/tb3_10/')
        rospy.Subscriber('/tb3_11/goal',Point,self.goal_listener,'/tb3_11/')
        rospy.Subscriber('/tb3_12/goal',Point,self.goal_listener,'/tb3_12/')
        rospy.Subscriber('/tb3_13/goal',Point,self.goal_listener,'/tb3_13/')
        rospy.Subscriber('/tb3_14/goal',Point,self.goal_listener,'/tb3_14/')
        rospy.Subscriber('/tb3_15/goal',Point,self.goal_listener,'/tb3_15/')
        rospy.Subscriber('/tb3_16/goal',Point,self.goal_listener,'/tb3_16/')
        rospy.Subscriber('/tb3_17/goal',Point,self.goal_listener,'/tb3_17/')
        rospy.Subscriber('/tb3_18/goal',Point,self.goal_listener,'/tb3_18/')
        rospy.Subscriber('/tb3_19/goal',Point,self.goal_listener,'/tb3_19/')

           
    def pose_listener(self,data):
        """ Odometry of all bots"""
        self.bot_id = data.bot_id
        bot_odom = data.botpose        
        
        for i,z in zip(self.bot_id,bot_odom):
            self.odom[i] = z
    
    def goal_listener(self,data,bot_id):
        self.goal[bot_id] = data        
    
if __name__ == '__main__':
    rospy.init_node('plotting_node',anonymous=True)

    y=yolo()
    #print(y.goal)
    rate = rospy.Rate(4) # 4hz    
    
    while not rospy.is_shutdown():
        plt.clf()        
        col = {'/tb3_0/':'r','/tb3_1/':'c','/tb3_2/':'b','/tb3_3/':'g','/tb3_4/':'y','/tb3_5/':'m','/tb3_6/':'k','/tb3_7/':'tab:orange', '/tb3_8/':'tab:purple','/tb3_9/':'tab:olive','/tb3_10/':'tab:gray','/tb3_11/':'tab:pink', '/tb3_12/':'#c9eb34','/tb3_13/':'tab:cyan','/tb3_14/':'aquamarine','/tb3_15/':'mediumseagreen','/tb3_16/':'#FC5A50','/tb3_17/':'#DDA0DD','/tb3_18/':'#FBDD7E','/tb3_19/':'#DBB40C'}
        try:
            for i in y.bot_id:
                euler = euler_from_quaternion([y.odom[i].pose.pose.orientation.x,y.odom[i].pose.pose.orientation.y,y.odom[i].pose.pose.orientation.z,y.odom[i].pose.pose.orientation.w])        
                yaw = euler[2]
                if yaw < 0:
                    yaw += 2 * pi
                plt.plot(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,"o",markersize=100, alpha=0.2,color=col[i])
                plt.plot(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,"o",color=col[i])
                plt.plot(y.goal[i].x,y.goal[i].y,"x",color=col[i])
                plt.quiver(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,cos(yaw),sin(yaw),units='xy',width=0.05,headwidth=2.,headlength=1.,color=col[i])
        except KeyError:
            print("chalna aage")
        plt.ylim([-20,20])
        plt.xlim([-20,20])
        plt.pause(0.01)        
        rate.sleep()