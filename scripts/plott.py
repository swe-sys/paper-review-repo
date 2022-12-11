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

import matplotlib.pyplot as plt


class yolo():
    def __init__(self):
        self.odom = {}
        self.goal = {}
        self.bot_id = []
        rospy.Subscriber('/obs_data', botPose, self.pose_listener)
        rospy.Subscriber('/tb3_0/goal',Point,self.goal_listener,'/tb3_0/')
        rospy.Subscriber('/tb3_1/goal',Point,self.goal_listener,'/tb3_1/')
        rospy.Subscriber('/tb3_2/goal',Point,self.goal_listener,'/tb3_2/')
        rospy.Subscriber('/tb3_3/goal',Point,self.goal_listener,'/tb3_3/')
        rospy.Subscriber('/tb3_4/goal',Point,self.goal_listener,'/tb3_4/')
        rospy.Subscriber('/tb3_5/goal',Point,self.goal_listener,'/tb3_5/')
        # rospy.Timer(rospy.Duration(0.2),self.plotter)


    def pose_listener(self,data):
        """ Odometry of all bots"""
        self.bot_id = data.bot_id
        bot_odom = data.botpose
        for i,z in zip(self.bot_id,bot_odom):
            self.odom[i] = z
    
    def goal_listener(self,data,bot_id):
        self.goal[bot_id] = data

    def plotter(self,event):
        for i in self.bot_id:
            plt.plot(self.odom[i].pose.pose.position.x,self.odom[i].pose.pose.position.y,"s")
            plt.plot(self.goal[i].x,self.goal[i].y,"x")
    
if __name__ == '__main__':
    rospy.init_node('plotting_node',anonymous=True)

    y=yolo()
    print(y.goal)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        plt.clf()
        # fig.canvas.draw()
        # fig.canvas
        col = {'/tb3_0/':'r','/tb3_2/':'b','/tb3_3/':'g','/tb3_4/':'y','/tb3_5/':'k','/tb3_1/':'c'}
        
        try:
            for i in y.bot_id:
                plt.plot(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,"o",markersize=125, alpha=0.3,color=col[i])
                plt.plot(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,"o",color=col[i])
                plt.plot(y.goal[i].x,y.goal[i].y,"x",color=col[i])
        except KeyError:
            print("chalna aage")
        #plt.legend(col.keys())
        plt.ylim([-10,10])
        plt.xlim([-10,10])
        
        plt.pause(0.01)
        rate.sleep()