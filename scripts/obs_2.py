#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi, radians, cos, sin , isinf
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from swarm_aggregation.msg import bot, botPose
import rospkg
import os

class obstacle(Pose2D):
    def __init__(self, x,y,theta,dist,min_dis):
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

                if prev_degree is None or abs(degree - prev_degree) <= 10:
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
        self.obs = []
        for i in range(len(self.ranges)):
            if not isinf(self.ranges[i]):
                # j = i - abs(self.ang_max)*180/pi
                pairs.append([i,self.ranges[i]])
        self.cones = self.split_list(pairs)
        # print(self.cones, "Cones", self.namespace)

        self.ratio = self.calculate_ratio(self.cones)
        # print(len(self.ratio), self.ratio, "Ratio", self.namespace)

        # try:
        #     for i in self.cones:
        #         angle = np.mean(np.array(i)[:,0])*(pi/180)
        #         distance = np.mean(np.array(i)[:,1])
        #         # print(distance, angle, "DA", self.namespace)
        #         min_dis = np.min(np.array(i)[:,1])
        #         if distance <= 3 and (-60*pi/180 < angle < 60*pi/180):
        #             obs_x = distance*cos(angle)
        #             obs_y = distance*sin(angle)
        #             global_x = self.x + (obs_x*cos(-self.yaw) + obs_y*sin(-self.yaw))
        #             global_y = self.y + (-obs_x*sin(-self.yaw) + obs_y*cos(-self.yaw))
        #             for x in self.ratio:
        #                 if x is not None:
        #                     if x <= 0.162:
        #                     # if (0.7 <= min(self.ranges[0:60]) <= 1.7 and 0.06 <= x <= 0.162) or (1.7 <  min(self.ranges[0:60]) <= 2.7 and 0.03 <= x < 0.06):
        #                         self.obs.append(obstacle(global_x,global_y,angle,distance,min_dis))
        #                     else:
        #                         self.obs = []
        #             # print(self.obs, self.namespace, 'Obs')            
        # except (IndexError):
        #     self.obs = []

        # print(self.obs, self.namespace, "Obs")                

    def calculate_ratio(self, lidar_data):
        ratio = []
        try:    
            for obj in lidar_data:
                if len(obj) >= 2:
                    a = obj[0][1]  # Starting range
                    b = obj[-1][1]  # Ending range
                    theta = abs(obj[0][0] - obj[-1][0])  # Absolute difference of starting and ending angle
                    
                    # Convert theta from degrees to radians
                    theta_rad = radians(theta)
                    
                    # Calculate c
                    # c = a**2 + b**2 - 2*a*b*cos(theta_rad)                    
                    
                    # Calculate the median distance to the object
                    distances = [reading[1] for reading in obj]
                    d = np.median(distances)
                    c = np.mean(distances)*theta_rad

                    # Calculate the ratio c/d
                    results = c / d
                    ratio.append(results)
                else:
                    ratio.append(None)  # Not enough data to calculate c and ratio
        except (IndexError):
            ratio = []

        return ratio                
            
    def wall_following(self):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        # print("Wall following")
        deg = 30
        dst = 0.5
        # while True:
        if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall
            self.speed.angular.z = -0.2
            self.speed.linear.x = 0.0
            # if min(self.ranges[deg:40]) <= dst: 
            #     self.speed.angular.z = -0.2 #left
            #     self.speed.linear.x = 0.0
            # elif min(self.ranges[320:340]) <= 0.6:
            #     self.speed.angular.z = 0.2 #right
            #     self.speed.linear.x = 0.0
            # print("Front Wall")                  
        elif min(self.ranges[deg:120]) < dst: # left wall 
            self.speed.angular.z = 0.0
            self.speed.linear.x = 0.2
            # print("Left wall")
        else:
            self.speed.angular.z = 0.1
            self.speed.linear.x = 0.2            
        # elif self.count >= 4:
        #     self.speed.angular.z = 0.01
        #     self.speed.linear.x = 0.1
        #     print("Phasa")    

    def set_goal(self):
        """ sets goal for bot"""         

        with open('{}/Data/Threshold_0.7/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("{},{},{},{},{}".format(rospy.get_time(),self.goal.x,self.goal.y,self.x, self.y) + '\n')

        # no_neigh = len(self.obs)
        # try:  
        #     if no_neigh >= 1:
        #         self.goal.x = (self.x + np.mean([i.x for i in self.obs]))/2
        #         self.goal.y = (self.y + np.mean([i.y for i in self.obs]))/2
        #     else:
        #         if self.goal.x == 3.0 and self.goal.y == -2.0:
        #             # print("called", self.namespace)
        #             self.goal.x = np.random.uniform(-1,6)
        #             self.goal.y = np.random.uniform(-6,-2)
        # except (AttributeError):
        #     print(AttributeError)

        
        for odom in self.bot_odom:
            self.disij.append(sqrt((odom.pose.pose.position.y - self.y)**2 + (odom.pose.pose.position.x - self.x)**2))
            self.delij.append(atan2((odom.pose.pose.position.y - self.y),(odom.pose.pose.position.x - self.x)))
            for i,m in enumerate(self.delij):
                if m < 0:
                    m += 2 * pi

        
        # Neighbour Set
        self.neigh = [odom for i,odom in enumerate(self.bot_odom) if self.disij[i] <= 3 and self.disij[i] > 0.1 and self.delij[i] <= pi/3 and self.delij[i] >= -pi/3 ]
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
        self.head_err = []
 
        self.set_goal()
        # self.ratio = self.calculate_ratio(self.cones)               
        self.incx = (self.goal.x - self.x)
        self.incy = (self.goal.y - self.y)

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))
        for i,b in enumerate(self.bearing):
            if b < 0:
                b += 2 * pi

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))        

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h

        for obs_element in self.obs:
            x_diff = obs_element.x -self.x
            y_diff = obs_element.y -self.y
            
            dist = sqrt(x_diff**2 + y_diff**2)
            ang = atan2(y_diff, x_diff)
            
            self.disij.append(dist)
            self.delij.append(ang)

        # Heading Error
        # self.head_err = self.bearing - self.yaw

        # self.head_err = (self.head_err + pi) % (2 * pi) - pi

        deg = 60
        dst = 3
        excluded_bot = [self.cur_bot_id_indx]

        if self.dis_err >= 0.885:
            if len(self.disij) == 0:
                self.speed.linear.x = 0.18
                self.speed.angular.z = K*np.sign(self.dtheta)
                # print("No neighbor", self.namespace)
            elif len(self.obs) == 0 and min(self.ranges[0:30]) <= 0.7 or min(self.ranges[330:360]) <= 0.7:
                self.wall_following()
                # print("Wall Following", self.namespace)
            for i,z in enumerate(self.disij):
                if z >= 0.7:
                    self.speed.linear.x = 0.18
                    self.speed.angular.z = K*np.sign(self.dtheta)
                    # print("Free", self.namespace)
                else:
                    t = rospy.get_time()
                    self.speed.linear.x = max((0.18 -(5000-t)*0.0001),0)                    
                    self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
                    # print("Engaged", self.namespace)
        else:
            if len(self.obs) >= 1:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                print("Aggregated", self.namespace)
            else:
                self.goal = Point(3.0,-2.0,0.0)
                # print("Alone", self.namespace)

        self.cmd_vel.publish(self.speed)
        self.pubg.publish(self.goal)

if __name__ == '__main__':
    k = 0
    l = [] #l is time
    rospy.init_node("Task2_controller")
    rate = rospy.Rate(4)
    bot = robot(6)
    rospy.sleep(10)     

    while not rospy.is_shutdown():
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)            
        rate.sleep()