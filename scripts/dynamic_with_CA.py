#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
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
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.pubo = rospy.Publisher('/'+self.namespace +'/odom2',Odometry,queue_size=10)
        self.pubg = rospy.Publisher('/goal', Point,queue_size=10)
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]
        self.disij = []
        self.delij = []
        self.neigh = []      
        self.bearing = [0]
        self.goal = Point()
        self.odom = Odometry()
        self.initial_no = -1
        
    def update_Odom(self,msg):
        """ Odometry of current bot"""
        self.odom = msg
        #print(type(msg))
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
        #print(self.bot_odom, '1')
        self.cur_bot_id_indx = bot_id.index(self.namespace)              

    def set_goal(self,r,ca):
        """ sets goal for bot"""
        self.neigh = []
        self.disij = []
        self.delij = []

        # Distance between bots   
        for odom in self.bot_odom:
            self.disij.append(sqrt((odom.pose.pose.position.y - self.y)**2 + (odom.pose.pose.position.x - self.x)**2))
            # print(odom.pose.pose.position.y,'y')
            self.delij.append(atan2((odom.pose.pose.position.y - self.y),(odom.pose.pose.position.x - self.x)))
            # print(self.delij)

        # Neighbour Set
        self.neigh = [odom for i,odom in enumerate(self.bot_odom) if self.disij[i] <= r and self.disij[i]>0.1 and self.delij[i] <= ca]
        self.neigh.append(self.odom)
        print(len(self.neigh),self.namespace)
        no_neigh = len(self.neigh)

        if no_neigh >= 2:
            self.initial_no = no_neigh
            #print(self.initial_no,no_neigh,"i and no")
            self.goal.x = np.mean([odom.pose.pose.position.x for odom in self.neigh])
            self.goal.y = np.mean([odom.pose.pose.position.y for odom in self.neigh])
        else:
            if self.goal.x == 0.0 and self.goal.y == 0.0:
                print("random goal alloted")
                self.goal.x = np.random.uniform(low= -10,high= 10)
                self.goal.y = np.random.uniform(low= -10,high= 10)
        
    def control(self,k):
        """control law for bot"""

        self.set_goal(3,pi/3)
        print(self.goal,'Goal')
        self.incx = (self.goal.x - self.x)
        self.incy = (self.goal.y - self.y)

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h

        if (self.dis_err) >= 0.75:
            temp = []
            vap = []
            for i,z in enumerate(self.disij):
                if len(self.neigh) >= 2 and 3>z>0.5:
                    v = 0.22
                    w = K*np.sign(self.dtheta)
                    print(z,self.delij[i]*(180/pi),'1')
                elif len(self.neigh) < 2 and z>3:
                    self.goal = Point(0,0,0)                              
                else:
                    t = rospy.get_time()
                    v = max((0.22-(100-t)*0.001),0)                    
                    w = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
                    temp.append(w)
                    vap.append(v)
            if temp:
                w = np.mean(temp)
                v = np.mean(vap)
                print(temp,vap,"temp_vap")            
        else:
            if len(self.neigh) == 1:
                print("Aas pass koi nahi!!")
                self.goal = Point(0,0,0)
            else:               
                v= 0.0
                w = 0.0
                print("Aggreated")

        self.speed.linear.x = v
        self.speed.angular.z = w        
        self.cmd_vel.publish(self.speed)
        self.pubg.publish(self.goal)

if __name__ == '__main__':
    k = 0
    l = [] #l is time
    rospy.init_node("Task2_controller")
    rate = rospy.Rate(4)
    bot = robot(6)

    while not rospy.is_shutdown() and k < 4000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)
        rate.sleep()