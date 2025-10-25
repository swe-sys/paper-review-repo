#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
import matplotlib.pyplot as plt
from swarm_aggregation.msg import bot, botPose
import rospkg

class robot(object):
    def __init__(self,no_of_bots):
        self.x = 0
        self.y = 0
        self.goal = Point()
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
        self.dis_err = 0 
        self.disij = []
        self.delij = []
        self.dtheta = []
        self.bearing = [0]
        self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        with open('{}/Data/Kmeans/{}.csv'.format(self.dirname,self.namespace.split("/")[1]),'a+') as f:
            f.write("time,goal_x,goal_y,x,y\n" )

        
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
        
        self.goal1 = Point()
        self.goal2 = Point()
        # self.goal = Point()
        self.goal1 = Point(-1.115,0.9075,0.0)
        self.goal2 = Point(2.185,2.455,0.0)
        self.incx1 = (self.goal1.x - self.x)        
        self.incy1 = (self.goal1.y - self.y)

        self.incx2 = (self.goal2.x - self.x)        
        self.incy2 = (self.goal2.y - self.y)
        
        # Distance Error
        self.dis_err1 = (sqrt(self.incx1**2+self.incy1**2))
        self.dis_err2 = (sqrt(self.incx2**2+self.incy2**2))

        if self.dis_err1 > self.dis_err2:
            self.goal = self.goal2
            # self.goal = (((self.goal2.x+self.x)/2), ((self.goal2.y+self.y)/2),0.0)
        else:
            self.goal = self.goal1
            # self.goal = (((self.goal1.x+self.x)/2), ((self.goal1.y+self.y)/2),0.0)              

    def control(self,k):
        """control law for bot"""        
        # Distance between goal and bot position
        self.set_goal()
        with open('{}/Data/Kmeans/{}.csv'.format(self.dirname,self.namespace.split("/")[1]),'a+') as f:
            f.write("{},{},{},{},{}".format(rospy.get_time(),self.goal.x,self.goal.y,self.x, self.y) + '\n')

        print(self.goal, self.namespace)
        self.incx = (self.goal.x - self.x)        
        self.incy = (self.goal.y - self.y) 

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))      
        self.dis_err = (sqrt(self.incx**2+self.incy**2))

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h        
        self.disij = []
        self.delij = []
        # Distance between bots       
        for odom in self.bot_odom:            
            self.disij.append(sqrt((odom.pose.pose.position.y - self.y)**2 + (odom.pose.pose.position.x - self.x)**2))
            # print(self.disij)
            self.delij.append(atan2((odom.pose.pose.position.y - self.y),(odom.pose.pose.position.x - self.x)))
            # print(self.delij)        
        if (self.dis_err) >= 1:
            temp = []
            vap = []
            for i,z in enumerate(self.disij):
                d = self.delij[i]
                if (z >= self.dis_err or d >= pi/3) or z == 0:
                    v = 0.18
                    w = K*np.sign(self.dtheta)
                    #print(z,self.delij[i]*(180/pi),'1')                                
                else:
                    t = rospy.get_time()
                    v = max((0.18-(100-t)*0.001),0)                    
                    w = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
            #         temp.append(w)
            #         vap.append(v)
            # if temp:
            #     w = np.mean(temp)
            #     v = np.mean(vap)
            #     print(temp,vap,"temp_vap")
        else:
            print("Clustered!!",self.namespace)
            v = 0.0
            w = 0.0        
        
        # print('W:',w,'v',v)
        self.speed.linear.x = v
        self.speed.angular.z = w
        self.cmd_vel.publish(self.speed)

if __name__ == '__main__':
    k = 0
    l = [] #l is time
    rospy.init_node("Multibot_controller")
    r = rospy.Rate(4)
    rospy.sleep(4)
    bot = robot(6)

    while not rospy.is_shutdown() and k < 4000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)
        # bot.cmd_vel.publish(bot.speed)  
        r.sleep()