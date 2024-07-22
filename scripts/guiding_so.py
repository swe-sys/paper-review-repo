#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi, radians, cos, sin , isinf
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from swarm_aggregation.msg import bot, botPose
import rospkg
import os

class robot(object):
    def __init__(self,no_of_bots): 
        self.x = 0
        self.y = 0
        self.ratio = 0
        self.bot_count = 0             
        self.cur_bot_id_indx = 0
        self.node_name = rospy.get_name()
        self.namespace = rospy.get_namespace()
        self.total_bots = no_of_bots
        self.speed = Twist()
        self.botpose = rospy.Subscriber('/obs_data',botPose,self.detect_bots)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.object_detector = rospy.Subscriber('/scan',LaserScan, self.obstacle_detector)
        self.pubg = rospy.Publisher('/goal', Point, queue_size=1)
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]
        self.count = 0
        self.disij = []
        self.delij = []
        self.neigh = []      
        self.bearing = [0]        
        self.ranges = LaserScan()
        self.goal = Point(np.random.uniform(-1,6), np.random.uniform(-6,-2), 0.0)
        self.odom = Odometry()
        self.initial_no = -1
        self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        self.iters = rospy.get_param("/iteration/")
        try:
            os.makedirs(f'{self.dirname}/Data/Threshold_0.7/Data{self.iters}')
        except FileExistsError:
            pass
        with open('{}/Data/Threshold_0.7/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("time, goal_x, goal_y, x, y \n" )
        
    def update_Odom(self,msg):
        """ Odometry of current bot"""
        self.odom = msg
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

    def split_list(self,input_list):                                                                                                                                                                                                                                                                                                                
        sublists = []
        sublist = []
        prev_degree = None
        try:
            for item in input_list:
                degree, _ = item                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

                if prev_degree is None or abs(degree - prev_degree) <= self.ang_inc*3:
                    sublist.append(item)
                else:
                    sublists.append(sublist)
                    sublist = [item]
                prev_degree = degree

            sublists.append(sublist)
        except (ValueError, TypeError) as e:
            print("Error occurred while splitting the list:", str(e))

        return sublists
    
    def obstacle_detector(self,msg):
        self.ranges = msg.ranges
        self.ang_max = msg.angle_max
        self.ang_inc = msg.angle_increment
        pairs = []
        for i in range(len(self.ranges)):
            if not isinf(self.ranges[i]):
                j = i - abs(self.ang_max)*180/pi
                pairs.append([self.ang_inc*j,self.ranges[i]])
        self.cones = self.split_list(pairs)                

    def calculate_ratio(self, lidar_data):
        self.ratio = []
            
        for obj in lidar_data:
            if len(obj) >= 2:
                a = obj[0][1]  # Starting range
                b = obj[-1][1]  # Ending range
                theta = abs(obj[0][0] - obj[-1][0])  # Absolute difference of starting and ending angle
                
                # Convert theta from degrees to radians
                theta_rad = radians(theta)
                
                # Calculate c
                c = a**2 + b**2 - 2*a*b*cos(theta_rad)
                
                # Calculate the median distance to the object
                distances = [reading[1] for reading in obj]
                d = np.median(distances)
                
                # Calculate the ratio c/d
                results = c / d
                self.ratio.append(results)
            else:
                self.ratio.append(None)  # Not enough data to calculate c and ratio
        
        return self.ratio                
            
    def wall_following(self):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        # print("Wall following")
        deg = 30
        dst = 0.7
        # while True:
        if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall  
            self.speed.angular.z = -0.2
            self.speed.linear.x = 0.0
            print("Front Wall")
            # if min(self.ranges[deg:40]) <= dst: 
            #     self.speed.angular.z = -0.2 #left
            #     self.speed.linear.x = 0.0
            #     print("left", self.namespace)
            # elif min(self.ranges[320:340]) < 0.6:
            #     self.speed.angular.z = 0.2 #right
            #     self.speed.linear.x = 0.0
            #     print("right", self.namespace)                 
        elif min(self.ranges[deg:120]) < dst: # left wall 
            self.speed.angular.z = 0.0
            self.speed.linear.x = 0.2
            print("Left wall", self.namespace)            
        # elif self.count >= 4:
        #     self.speed.angular.z = 0.01
        #     self.speed.linear.x = 0.1
        #     print("Phasa")    

    def set_goal(self,r,ca):
        """ sets goal for bot"""
        self.neigh = []
        self.disij = []
        self.delij = []

        with open('{}/Data/Threshold_0.7/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("{},{},{},{},{}".format(rospy.get_time(),self.goal.x,self.goal.y,self.x, self.y) + '\n')

        # Distance between bots   
        for odom in self.bot_odom:
            self.disij.append(sqrt((odom.pose.pose.position.y - self.y)**2 + (odom.pose.pose.position.x - self.x)**2))
            self.delij.append(atan2((odom.pose.pose.position.y - self.y),(odom.pose.pose.position.x - self.x)))
            # for i,m in enumerate(self.delij):
            #     if m < 0:
            #         m += 2 * pi

        # Neighbour Set
        self.neigh = [odom for i,odom in enumerate(self.bot_odom) if self.disij[i] <= r and self.disij[i] > 0.1 and self.delij[i] <= ca and self.delij[i] >= -ca ]
        self.neigh.append(self.odom)
        # print(len(self.neigh),self.namespace)
        no_neigh = len(self.neigh)

        if no_neigh >= 2:
            self.initial_no = no_neigh
            self.goal.x = np.mean([odom.pose.pose.position.x for odom in self.neigh])
            self.goal.y = np.mean([odom.pose.pose.position.y for odom in self.neigh])
        else:
            if self.goal.x == 3.0 and self.goal.y == -2.0:
                # print("called", self.namespace)
                self.goal.x = np.random.uniform(-1,6)
                self.goal.y = np.random.uniform(-6,-2)
        
    def control(self,k):
        """control law for bot"""
 
        self.set_goal(3,(pi/2))
        self.calculate_ratio(self.cones)               
        self.incx = (self.goal.x - self.x)
        self.incy = (self.goal.y - self.y)

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))
        # for i,b in enumerate(self.bearing):
        #     if b < 0:
        #         b += 2 * pi

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))        

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h

        deg = 30
        dst = 0.7

        if (self.dis_err) >= 0.85:
            temp = []
            vap = []
            excluded_bot = [self.cur_bot_id_indx]
            # print(self.cur_bot_id_indx)
            for i,z in enumerate(self.disij):
                if i not in excluded_bot:
                    if (min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst):
                        # or obstacle_distance <= obstacle_radius or wall_distance <= wall_radius 
                        if len(self.neigh) < 2:
                            self.wall_following()                                                                        
                            print("Wall Following", self.namespace, self.goal, len(self.neigh))
                        elif len(self.neigh) >= 2 and (min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst):
                            t = rospy.get_time()                       
                            self.speed.linear.x = max((0.12 -(4000-t)*0.00001),0)                    
                            self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])                  
                            print("Engaged", self.namespace, self.goal, len(self.neigh)) 
                    # elif  len(self.neigh) >= 2 and (min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst):
                    # #     # and (min(self.ranges[0:deg]) >= 1.25*dst or min(self.ranges[(359-deg):]) >= 1.25*dst)
                    # #     self.speed.linear.x = 0.18
                    # #     self.speed.angular.z = K*np.sign(self.dtheta)                        
                    # #     print("Free", self.namespace, self.goal, len(self.neigh))                                       
                    # elif z < 0.7:                        
                    #     t = rospy.get_time()                       
                    #     self.speed.linear.x = max((0.12 -(4000-t)*0.00001),0)                    
                    #     self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])                  
                    #     print("Engaged", self.namespace, self.goal, len(self.neigh))
                    else:
                    #     # and (min(self.ranges[0:deg]) >= 1.25*dst or min(self.ranges[(359-deg):]) >= 1.25*dst)
                        self.speed.linear.x = 0.18
                        self.speed.angular.z = K*np.sign(self.dtheta)                        
                        print("Free", self.namespace, self.goal, len(self.neigh))                       
                # if temp:
                #     self.speed.angular.z = np.mean(temp)
                #     self.speed.linear.x = np.mean(vap)   #/len(self.neigh)                
        else:
            if len(self.neigh) < 2:
                # or (min(self.ranges[0:deg]) <= 1.0 or min(self.ranges[(359-deg):]) <= 1.0)
                self.goal = Point(3.0, -2.0, 0.0)
                self.set_goal(3,(pi/2))
                # self.wall_following()
                print("Alone", self.namespace, self.dis_err, len(self.neigh))                                            
            else:                
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0        
                print("Aggreated",self.namespace, self.dis_err, len(self.neigh))

        self.cmd_vel.publish(self.speed)
        self.pubg.publish(self.goal)

if __name__ == '__main__':
    k = 0
    l = [] #l is time
    rospy.init_node("Task2_controller")
    rate = rospy.Rate(4)
    bot = robot(9)
    rospy.sleep(10)     

    while not rospy.is_shutdown():
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)            
        rate.sleep()