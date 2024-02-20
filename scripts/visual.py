#!/usr/bin/env python3
from __future__ import division
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from swarm_aggregation.msg import obs
from math import cos, sin, pi ,isinf
import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.ion() 

class visual():
    def __init__(self):
        self.odom = {}
        self.goal = {}
        self.bot_id = []
        self.radius = {}
        self.balma = {}        
        #rospy.Subscriber('/obs_data', botPose, self.pose_listener)
        # rospy.Subscriber('/obs_data', botPose, self.store_data)
        rospy.Subscriber('/tb3_1/odom',Odometry,self.odom_listener,'/tb3_1/')
        rospy.Subscriber('/tb3_2/odom',Odometry,self.odom_listener,'/tb3_2/')
        rospy.Subscriber('/tb3_3/odom',Odometry,self.odom_listener,'/tb3_3/')
        rospy.Subscriber('/tb3_4/odom',Odometry,self.odom_listener,'/tb3_4/')
        rospy.Subscriber('/tb3_5/odom',Odometry,self.odom_listener,'/tb3_5/')
        rospy.Subscriber('/tb3_6/odom',Odometry,self.odom_listener,'/tb3_6/')

        rospy.Subscriber('/tb3_1/goal',Point,self.goal_listener,'/tb3_1/')
        rospy.Subscriber('/tb3_2/goal',Point,self.goal_listener,'/tb3_2/')
        rospy.Subscriber('/tb3_3/goal',Point,self.goal_listener,'/tb3_3/')
        rospy.Subscriber('/tb3_4/goal',Point,self.goal_listener,'/tb3_4/')
        rospy.Subscriber('/tb3_5/goal',Point,self.goal_listener,'/tb3_5/')
        rospy.Subscriber('/tb3_6/goal',Point,self.goal_listener,'/tb3_6/')
        
        rospy.Subscriber('/tb3_1/radius',Point,self.radius_listener,'/tb3_1/')
        rospy.Subscriber('/tb3_2/radius',Point,self.radius_listener,'/tb3_2/')
        rospy.Subscriber('/tb3_3/radius',Point,self.radius_listener,'/tb3_3/')
        rospy.Subscriber('/tb3_4/radius',Point,self.radius_listener,'/tb3_4/')
        rospy.Subscriber('/tb3_5/radius',Point,self.radius_listener,'/tb3_5/')
        rospy.Subscriber('/tb3_6/radius',Point,self.radius_listener,'/tb3_6/')

        
        rospy.Subscriber('/tb3_1/obs',obs,self.obs_listener,'/tb3_1/')
        rospy.Subscriber('/tb3_2/obs',obs,self.obs_listener,'/tb3_2/')
        rospy.Subscriber('/tb3_3/obs',obs,self.obs_listener,'/tb3_3/')
        rospy.Subscriber('/tb3_4/obs',obs,self.obs_listener,'/tb3_4/')
        rospy.Subscriber('/tb3_5/obs',obs,self.obs_listener,'/tb3_5/')
        rospy.Subscriber('/tb3_6/obs',obs,self.obs_listener,'/tb3_6/')

    def odom_listener(self,data,bot_id):
        """ Odometry of all bots"""
        self.odom[bot_id] = data

    def goal_listener(self,data,bot_id):
        self.goal[bot_id] = data

    def radius_listener(self,data,bot_id):
        self.radius[bot_id] = data
    
    def obs_listener(self,data,bot_id):
        self.balma[bot_id] = data.obspose
    
if __name__ == '__main__':
    rospy.init_node('plotting_node',anonymous=True)

    y = visual()
    #print(y.goal)
    rate = rospy.Rate(4) # 4hz    
    rospy.sleep(2)
    fig = plt.figure()
    ax = fig.add_subplot()
    # fig.ax
    while not rospy.is_shutdown():
        #fig.clf()
        ax.clear()       
        col = {'/tb3_1/':'r','/tb3_2/':'c','/tb3_3/':'b','/tb3_4/':'g','/tb3_5/':'y','/tb3_6/':'m'}
        #,'/tb3_6/':'k','/tb3_7/':'tab:orange', '/tb3_8/':'tab:purple',
        # '/tb3_9/':'tab:olive','/tb3_10/':'tab:gray','/tb3_11/':'tab:pink',
        # '/tb3_12/':'#c9eb34','/tb3_13/':'tab:cyan','/tb3_14/':'aquamarine',
        # '/tb3_15/':'mediumseagreen','/tb3_16/':'#FC5A50','/tb3_17/':'#DDA0DD',
        # '/tb3_18/':'#FBDD7E','/tb3_19/':'#DBB40C'}
        y.bot_id = list(col.keys())
        try:            
            for i in y.bot_id:
                # euler = euler_from_quaternion([y.odom[i].pose.pose.orientation.x,y.odom[i].pose.pose.orientation.y,y.odom[i].pose.pose.orientation.z,y.odom[i].pose.pose.orientation.w])        
                # yaw = euler[2]
                # if yaw < 0:
                #     yaw += 2 * pi
                # plt.plot(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,"o",markersize=100, alpha=0.2,color=col[i])
                ax.plot(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,"o",color=col[i])
                ax.plot(y.goal[i].x,y.goal[i].y,"x",color=col[i])
                # for j in y.balma[i]:
                #     ax.plot(j.x, j.y,'*',markersize=10,color=col[i])
                # p = patches.Circle((y.goal[i].x,y.goal[i].y), radius=y.radius[i].x,fill=False)
                # ax.add_patch(p)
                #plt.quiver(y.odom[i].pose.pose.position.x,y.odom[i].pose.pose.position.y,cos(yaw),sin(yaw),units='xy',width=0.05,headwidth=2.,headlength=1.,color=col[i])
        except KeyError:
            print("chalna aage")
        ax.legend(col)
        plt.ylim([-10,10])
        plt.xlim([-10,10])
        plt.pause(0.1)        
        rate.sleep()