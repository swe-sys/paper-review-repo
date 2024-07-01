#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi, sin, cos, isinf, isnan, radians
#import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from swarm_aggregation.msg import bot, obs
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
        self.cur_bot_id_indx = 0
        self.ratio = 0
        self.obsplot = obs()
        self.node_name = rospy.get_name()
        self.namespace = rospy.get_namespace()
        self.total_bots = no_of_bots
        self.speed = Twist()
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]
        self.disij = []
        self.delij = []
        self.neigh = []      
        self.bearing = [0]
        self.ranges = LaserScan()
        self.goal = Point(np.random.uniform(1,6), np.random.uniform(-6,0), 0.0)
        self.odom = Odometry()
        self.count = 0
        self.initial_no = -1

        self.object_detector = rospy.Subscriber('/scan',LaserScan, self.scanner)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom) 

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)    
        self.pubg = rospy.Publisher('/goal', Point, queue_size=1)
        self.rad_pub = rospy.Publisher('/radius', Point, queue_size=1)
        self.obs_pub = rospy.Publisher('/obs',obs, queue_size=1)

        self.iter = rospy.get_param("/iteration/")
        self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        try:
            os.makedirs(f'{self.dirname}/Data/Data{self.iter}')
        except FileExistsError:
            pass
        rospy.sleep(2)
        with open('{}/Data/Data{}/{}.csv'.format(self.dirname,self.iter,self.namespace.split("/")[1]),'a+') as f:
            f.write("time, goal_x, goal_y, x, y \n" )
        
    def update_Odom(self,odom):
        """ Odometry of current bot"""
        self.x = odom.pose.pose.position.x 
        self.y = odom.pose.pose.position.y
        self.rot_q = odom.pose.pose.orientation
        euler = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])
        self.yaw = atan2(sin(euler[2]),cos(euler[2])) 
        self.odom = Pose2D(self.x,self.y,self.yaw)
        self.vel = Pose2D(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z)   

    # def detect_bots(self,data):
    #     """ Odometry of all bots"""
    #     bot_id = data.bot_id
    #     odoms = data.botpose
    #     self.bot_odom = odoms
    #     self.cur_bot_id_indx = bot_id.index(self.namespace)

    def find_patches(self, data, max_patch_length=28, diff_threshold=0.1):
        patches = []
        current_patch = []
        start_indices = []

        for i in range(1, len(data)):
            if abs(data[i] - data[i - 1]) > diff_threshold:
                if current_patch:
                    if len(current_patch) <= max_patch_length:
                        patches.append((current_patch, start_indices[0]))
                    current_patch = []
                    start_indices = []
            current_patch.append(data[i])
            start_indices.append(i)

        # Append the last patch if it exists and meets the length criteria
        if current_patch and len(current_patch) <= max_patch_length:
            patches.append((current_patch, start_indices[0]))

        return patches, len(patches)
    
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
    
    def scanner(self,msg):       
        self.ranges = msg.ranges
        self.ang_min = msg.angle_min
        self.ang_max = msg.angle_max
        self.ang_inc = msg.angle_increment
        self.obs = []
        pairs = []
        sobs = []

        patches, count = self.find_patches(msg.ranges, diff_threshold=0.1)        
        for i, (patch, start_index) in enumerate(patches):
            patch_no_a = patch[0]
            patch_no_b = patch[-1]
            patch_no_distance = patch[len(patch) // 2]
            deg = (len(patch) )/3.0

            # Calculate the start angle in radians
            start_angle = start_index * self.ang_inc + self.ang_min
            # Apply cosine rule
            c_squared = patch_no_a ** 2 + patch_no_b ** 2 - 2 * patch_no_a * patch_no_b * cos(radians(deg))
            c = sqrt(c_squared)
            self.ratio = c / patch_no_distance 

        # creating the pairs
        for i in range(len(self.ranges)):
            if not isinf(self.ranges[i]):
                j = i-abs(self.ang_max)*180/pi
                pairs.append([self.ang_inc*j,self.ranges[i]]) 
                
        self.cones = self.split_list(pairs)        
        try:
            for i in self.cones:
                angle = np.mean(np.array(i)[:,0])*(pi/180)
                distance = np.mean(np.array(i)[:,1])
                min_dis = np.min(np.array(i)[:,1])
                if 0.7 < distance < 2.7 and (-60*pi/180 < angle < 60*pi/180):
                    obs_x = distance*cos(angle)
                    obs_y = distance*sin(angle)
                    global_x = self.x + (obs_x*cos(-self.yaw) + obs_y*sin(-self.yaw))
                    global_y = self.y + (-obs_x*sin(-self.yaw) + obs_y*cos(-self.yaw))
                    self.obs.append(obstacle(global_x,global_y,angle,distance,min_dis))
                    print(self.obs, self.namespace, 'Obs', self.ratio)
                    sobs.append(Point(global_x,global_y,angle)) 
                    self.obsplot.obspose = sobs        
            # self.hist.append([self.obs])
        except (IndexError):
            self.obs = []     
    
    def wall_following(self):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        # print("Wall following")
        deg = 30
        dst = 0.65
        # while True:
        if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall  
            self.speed.angular.z = -0.2
            self.speed.linear.x = 0.0                   
        elif min(self.ranges[deg:60]) < dst: # left wall 
            self.speed.angular.z = 0.0
            self.speed.linear.x = 0.2             
        # elif self.count >= 3:
        #     self.speed.angular.z = 0.01
        #     self.speed.linear.x = 0.1                       

    def set_goal(self):
        """ sets goal for bot"""
        no_neigh = len(self.obs)
        try:  
            if no_neigh >= 1: #and not random:
                self.goal.x = (self.x + np.mean([i.x for i in self.obs]))/2
                self.goal.y = (self.y + np.mean([i.y for i in self.obs]))/2                
            else :
                if self.goal.x == 3.0 and self.goal.y == -2.0:
                    print("andar gya")
                    self.goal.x = np.random.uniform(1,6)
                    self.goal.y = np.random.uniform(-6,0)
        except (AttributeError):
            print(AttributeError)
        
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

        for obs_element in self.obs:
            x_diff = obs_element.x -self.x
            y_diff = obs_element.y -self.y
            
            dist = sqrt(x_diff**2 + y_diff**2)
            ang = atan2(y_diff, x_diff)
            
            self.disij.append(dist)
            self.delij.append(ang)

        # Define wall positions
        # wall_positions = [(-10.578274, self.y), (8.187240, self.y), (self.x < 0.9, -3.801140), (self.x, 3.665870), (self.x > 0.0, 0.820250), (self.x, -6.576820), (0.477467, self.y > 0.5), (0.422073, self.y < -3.25)]  # Example wall positions
        # # wall_positions = [(-7.25, self.y), (8.0000, self.y), (self.x, -3.65), (self.x, 3.40), (self.x, 0.65), (self.x, -6.25), (0.0, self.y), (0.9, self.y)]
        # wall_radius = 0.65
        deg = 30
        dst = 0.7

        #Static Obstacles
        # obstacle_positions = [(-5.50, 1.50), (-2.20, 1.50), (-4.0,0.0), (-5.50,-1.50), (-2.20,-1.50)] 
        # obstacle_radius = 0.65

        # Combine wall positions with other obstacles        
        # obstacle_distance = min(np.linalg.norm(np.array([self.x, self.y]) - np.array(obstacle)) for obstacle in obstacle_positions)
        # wall_distance = min(np.linalg.norm(np.array([self.x, self.y]) - np.array(wall)) for wall in wall_positions)   
        
        # if (self.dis_err) >= 0.850:
        #     if len(self.disij) == 0:
        #         self.speed.linear.x = 0.18
        #         self.speed.angular.z = K*np.sign(self.dtheta)
        #         print("Free", self.namespace, self.goal)
        #     else:
        #         if min(self.ranges[0:30]) <= 0.65 and ((self.ratio > 0.2) or isnan(self.ratio)):                    
        #             self.wall_following()
        #             self.goal = Point(3.0, -2.0, 0.0)
        #             print("Obstacles", self.namespace, self.goal)
        #         else:
        #             # self.set_goal()
        #             for i,z in enumerate(self.disij):
        #                 if z >= 0.65:
        #                     self.speed.linear.x = 0.18
        #                     self.speed.angular.z = K*np.sign(self.dtheta)
        #                     print("New Goal", self.namespace, self.goal)
        #                 else:
        #                     t = rospy.get_time()
        #                     self.speed.linear.x = max((0.18 -(5000-t)*0.0001),0)                    
        #                     self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
        #                     print("Engaged", self.namespace, self.goal)
        # else:
        #     if len(self.obs) >= 2: 
        #         self.speed.linear.x = 0.0
        #         self.speed.angular.z = 0.0
        #         print("Aggreated", self.namespace, self.goal)                
        #     else:               
        #         # self.set_goal()
        #         self.goal = Point(3.0,-2.0,0.0)
        #         print("Alone", self.namespace, self.goal)

        if (self.dis_err) >= 1:
            excluded_bot = [self.cur_bot_id_indx]            
            # print(self.cur_bot_id_indx)
            for i,z in enumerate(self.disij):
                if i not in excluded_bot:
                    # if (min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst or obstacle_distance <= obstacle_radius or wall_distance <= wall_radius) and z >= 0.65:
                    if (min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst) and z >= 0.65:  
                        self.count += 1                        
                        self.goal = Point(3.0, -2.0, 0.0)
                        self.wall_following()                        
                        print("Wall Following", self.ratio, z, self.namespace, self.dis_err)
                    # elif z >= 0.65 and obstacle_distance > obstacle_radius:
                    elif z >= 0.65 and (min(self.ranges[0:deg]) > dst*1.25 or min(self.ranges[(359-deg):]) > dst*1.25):
                        self.set_goal()
                        self.speed.linear.x = 0.18
                        self.speed.angular.z = K*np.sign(self.dtheta)
                        # print(z,self.delij[i]*(180/pi),i,self.namespace,'1',self.goal, obstacle_distance)
                        print("Free", self.namespace, self.ratio, z, self.dis_err)                                        
                    else:                        
                        t = rospy.get_time()                       
                        self.speed.linear.x = max((0.12 -(4000-t)*0.00001),0)                    
                        self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
                        # temp.append(self.speed.angular.z)
                        # vap.append(self.speed.linear.x)
                        print("Engaged", self.namespace, z, i)
                    # print(z,self.delij[i]*(180/pi),i,self.namespace,'2',self.goal, obstacle_distance)
                # if temp:
                #     self.speed.angular.z = np.mean(temp)
                #     self.speed.linear.x = np.mean(vap)   #/len(self.neigh)
        else:                
            if len(self.obs) >= 2: 
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                print("Aggreated", self.namespace, self.goal)                
            else:               
                # self.set_goal()
                self.goal = Point(3.0,-2.0,0.0)
                print("Alone", self.namespace, self.goal)             


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