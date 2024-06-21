#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np
from collections import deque
from math import *
from swarm_aggregation.msg import obs
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class obstacle(Pose2D):
    def __init__(self,x,y,theta,dist,min_dis):
        super().__init__(x,y,theta)
        self.static = False
        self.detected_time = rospy.get_time()
        self.dist = dist
        self.min_dis = min_dis
    
    def __eq__(self,other):
        if abs(self.x - other.x)< 0.1 :
            if abs(self.y - other.y)< 0.1 :
                return True
            else:
                return False
        else:
            return False

class robot(object):
    def __init__(self,no_of_bots): 
        self.total_bots = no_of_bots 
        self.x = 0
        self.y = 0        
        self.incident_time = []           
        self.yaw = 0
        self.range = [0]
        self.time = 0
        self.angle = 0
        self.ang_max = 0
        self.ang_inc = 0
        self.disij = []
        self.delij = []             
        self.bearing = [0]
        self.neigh = 0
        self.robot = []
        self.safe_zone = [0,0,0]
        self.initial_no = -1                      
        self.goal = Point(np.random.uniform(-6,6), np.random.uniform(-6,6), 0.0)
        self.odom = Odometry()
        self.namespace = rospy.get_namespace()
        self.speed = Twist()
        self.hist = deque(maxlen=20)

        self.object_detector = rospy.Subscriber('/scan',LaserScan, self.scanner)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom) 

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)    
        self.pubg = rospy.Publisher('/goal', Point, queue_size=1)
        self.rad_pub = rospy.Publisher('/radius', Point, queue_size=1)
        self.obs_pub = rospy.Publisher('/obs',obs, queue_size=1)      
        
        
    def update_Odom(self,odom):
        """ Odometry of current bot"""        
        self.x = odom.pose.pose.position.x 
        self.y = odom.pose.pose.position.y
        self.rot_q = odom.pose.pose.orientation
        euler = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])
        self.yaw = atan2(sin(euler[2]),cos(euler[2])) 
        self.odom = Pose2D(self.x,self.y,self.yaw)
        self.vel = Pose2D(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z)

    def scanner(self,msg):       
        self.range = msg.ranges
        self.ang_max = msg.angle_max
        self.ang_inc = msg.angle_increment
        self.obs = []
        self.obs_up = []
        self.obs_down = []
        pairs = []           
        # creating the pairs
        for i in range(len(self.range)):
            if not isinf(self.range[i]):
                j=i-abs(self.ang_max)*180/pi
                pairs.append([self.ang_inc*j,self.range[i]])                     

        self.cones = self.split_list(pairs)        
        try:
            for i in self.cones:
                angle = np.mean(np.array(i)[:,0])*(pi/180)
                distance = np.mean(np.array(i)[:,1])
                min_dis = np.min(np.array(i)[:,1])
                if distance < 3:
                    obs_x = distance*cos(angle)
                    obs_y = distance*sin(angle)
                    global_x = self.x + (obs_x*cos(-self.yaw) + obs_y*sin(-self.yaw))
                    global_y = self.y + (-obs_x*sin(-self.yaw) + obs_y*cos(-self.yaw))
                    self.obs.append(obstacle(global_x,global_y,angle,distance,min_dis))
                    print(self.obs, self.namespace, 'Obs')
                if distance < 3 and (-90*pi/180 < angle < 90*pi/180):
                    obs_x = distance*cos(angle)
                    obs_y = distance*sin(angle)
                    global_x = self.x + (obs_x*cos(-self.yaw) + obs_y*sin(-self.yaw))
                    global_y = self.y + (-obs_x*sin(-self.yaw) + obs_y*cos(-self.yaw))
                    self.obs_up.append(obstacle(global_x,global_y,angle,distance,min_dis))
                    print(self.obs_up, self.namespace, 'Obs_up')
                if distance < 3 and (90*pi/180 < angle < -90*pi/180):
                    obs_x = distance*cos(angle)
                    obs_y = distance*sin(angle)
                    global_x = self.x + (obs_x*cos(-self.yaw) + obs_y*sin(-self.yaw))
                    global_y = self.y + (-obs_x*sin(-self.yaw) + obs_y*cos(-self.yaw))
                    self.obs_up.append(obstacle(global_x,global_y,angle,distance,min_dis))
                    print(self.obs_down, self.namespace, 'Obs_down')                            
            # self.hist.append([self.obs])
        except (IndexError):
            self.obs = []
            self.obs_up = []
            self.obs_down = []

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
    
    def set_goal(self): #,random=False
        """outputs required = goal, input = neighbour set, using mean(self+ neigh       bour_set/2)"""
        # self.scanner()
        # with open('{}/Data/{}.csv'.format(self.dirname,self.namespace.split("/")[1]),'a+') as f:
        #     f.write("{},{},{},{},{}".format(rospy.get_time(),self.goal.x,self.goal.y,self.x, self.y) + '\n')

        no_neigh = len(self.obs)
        try:  
            if no_neigh >= 1: #and not random:
                self.goal.x = (self.x + np.mean([i.x for i in self.obs]))/2
                self.goal.y = (self.y + np.mean([i.y for i in self.obs]))/2                
            else :
                if self.goal.x == 0 and self.goal.y == 0:
                    print("andar gya")
                    self.goal.x = np.random.uniform(-6,6)
                    self.goal.y = np.random.uniform(-6,6)                
        except (AttributeError):
            print(AttributeError)
        # print(self.goal,"goal",self.namespace)

    def control(self,k):
        self.set_goal()
        self.incx = (self.goal.x - self.x)
        self.incy = (self.goal.y - self.y)

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))
        #print(self.dis_err)        

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h

        for obs_element in self.obs:
            x_diff = obs_element.x -self.x
            y_diff = obs_element.y -self.y
            
            dist = sqrt(x_diff**2 + y_diff**2)
            ang = atan2(y_diff, x_diff)
            
            self.disij.append(dist)
            self.delij.append(ang)

        if (self.dis_err) >= 0.850:
            n = len(self.obs_up)
            p = len(self.obs_down)
            if n == 0:
                self.speed.linear.x = 0.18
                self.speed.angular.z = K*np.sign(self.dtheta)
                print(self.goal,self.namespace,"Goal Free")
            else:
                for i,z in enumerate(self.disij):                
                    if z >= 0.75:                       
                        error = np.sum([i.theta for i in self.obs_up])-(n)*self.yaw                                       
                        u = (1/n)*error                                               
                        self.speed.angular.z = u
                        self.speed.linear.x = 0.1
                        print("Flock",self.namespace)                        
                    else: 
                        t = rospy.get_time()
                        self.speed.linear.x = max((0.18 -(5000-t)*0.0001),0)                    
                        self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
                        print("avoid")
            if p >= 1:
                self.speed.linear.x = 0.18
                self.speed.angular.z = K*np.sign(self.dtheta)
        else:
            if len(self.obs) >= 2:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                print("Converged", self.namespace)
            else:
                self.goal.x = np.random.uniform(-6,6)
                self.goal.y = np.random.uniform(-6,6)

        self.cmd_vel.publish(self.speed)

if __name__ =='__main__':
    rospy.init_node("Synchronization")    
    rospy.loginfo("Chal Gye badde")
    k = 0
    l = [] #l is time
    rate = rospy.Rate(4)
    bot = robot(6)     
    rospy.sleep(6)
    # bot.set_goal()
    while not rospy.is_shutdown() and k < 500000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)            
        rate.sleep()           
