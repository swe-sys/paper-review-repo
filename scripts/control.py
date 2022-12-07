#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
import matplotlib.pyplot as plt
from swarm_aggregation.msg import bot, botPose

class robot(object):
    def __init__(self,no_of_bots):
        self.x = 0
        self.y = 0
        self.cur_bot_id_indx = 0
        self.node_name = rospy.get_name()
        self.namespace = rospy.get_namespace()
        self.total_bots = no_of_bots
        self.speed = Twist()
        self.botpose = rospy.Subscriber('/obs_data',botPose,self.detect_bots)
        self.odom = rospy.Subscriber("/odom",Odometry,self.update_Odom)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]
        self.disij = []
        self.delij = []
        self.neigh = []      
        self.bearing = [0]
        
    def update_Odom(self,msg):
        """ Odometry of current bot"""
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y
        self.rot_q = msg.pose.pose.orientation
        euler = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])
        
        self.yaw = euler[2]
        if self.yaw < 0:
            self.yaw += 2 * pi

    def detect_bots(self,data):
        """ Odometry of all bots"""
        bot_id = data.bot_id
        odoms = data.botpose
        self.bot_odom = odoms
        self.cur_bot_id_indx = bot_id.index(self.namespace)                 

    def set_goal(self):
        """ sets goal for bot"""
        self.neigh = []

        # Distance between bots   
        for odom in self.bot_odom:            
            self.disij.append(sqrt((odom.pose.pose.position.y - self.y)**2 + (odom.pose.pose.position.x - self.x)**2))
            # print(self.disij)
            self.delij.append(atan2((odom.pose.pose.position.y - self.y),(odom.pose.pose.position.x - self.x)))
            # print(self.delij)
            self.goal = Point()
            if (self.disij) <= 4:
                self.neigh = self.disij
                c = sum(self.neigh)/len(self.neigh)
                self.goal = Point(self.x+c,self.y+c,0.0)
            else:
                self.goal = Point(self.x,self.y,0.0)              

    def control(self,k):
        """control law for bot"""
        self.set_goal()
        self.incx = (self.goal.x - self.x)        
        self.incy = (self.goal.y - self.y)

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))
        
        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))
        
        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h    
               
        self.disij = []
        self.delij = []       
            
        if (self.dis_err) >= 0.1:
            print(self.dis_err,'distance error')
            self.speed.linear.x  = 0.22
            self.speed.angular.z = K*np.sign(self.dtheta)
            print(self.speed.angular.z,'Angular')
        else:
            print('Done!!')
            print(self.dis_err,'distance error')
            self.speed.linear.x = 0
            self.speed.angular.z = 0 
        
        self.cmd_vel.publish(self.speed)

if __name__ == '__main__':
    k = 0
    l = [] #l is time
    rospy.init_node("Task2_controller")
    r = rospy.Rate(4)
    bot = robot(5)

    while not rospy.is_shutdown() and k < 4000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)
        r.sleep()