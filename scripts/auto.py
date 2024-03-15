#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from swarm_aggregation.msg import bot, botPose
import rospkg

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
        self.obstacle_detector = rospy.Subscriber('/scan',LaserScan, self.obstacle_detector)
        #self.pubo = rospy.Publisher('/'+self.namespace +'/odom2',Odometry,queue_size=10)
        self.pubg = rospy.Publisher('/goal', Point,queue_size=10)
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]
        self.disij = []
        self.delij = []
        self.neigh = []      
        self.bearing = [0]
        self.ranges = LaserScan()
        self.goal = Point(np.random.uniform(1,6), np.random.uniform(-6,0), 0.0)
        self.odom = Odometry()
        self.initial_no = -1
        # self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        # with open('{}/scripts/{}.csv'.format(self.dirname,self.namespace.split("/")[1]),'a+') as f:
        #     f.write("time, goal_x, goal_y, x, y \n" )
        
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

    def obstacle_detector(self,msg):
        self.ranges = msg.ranges
    
    def wall_following(self):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        # print("Wall following")
        deg = 50
        dst = 0.5
        # while True:
        if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall  
            self.speed.angular.z = 0.2
            self.speed.linear.x = 0.0                   
        elif min(self.ranges[deg:120]) < dst: # left wall 
            self.speed.angular.z = 0.0
            self.speed.linear.x = 0.2            
        #elif abs(dist([self.x,self.y],[self.goal.x,self.goal.y]) - min(self.odom_counter)) < 0.17:
            #self.go2goal()      
        else:
            self.speed.angular.z = 0.1
            self.speed.linear.x = 0.02                       

    def set_goal(self,r,ca):
        """ sets goal for bot"""
        self.neigh = []
        self.disij = []
        self.delij = []

        # with open('{}/scripts/{}.csv'.format(self.dirname,self.namespace.split("/")[1]),'a+') as f:
        #     f.write("{},{},{},{},{}".format(rospy.get_time(),self.goal.x,self.goal.y,self.x, self.y) + '\n')

        # Distance between bots   
        for odom in self.bot_odom:
            self.disij.append(sqrt((odom.pose.pose.position.y - self.y)**2 + (odom.pose.pose.position.x - self.x)**2))
            self.delij.append(atan2((odom.pose.pose.position.y - self.y),(odom.pose.pose.position.x - self.x)))
            # for i,m in enumerate(self.delij):
            #     if m < 0:
            #         m += 2 * pi

        # Neighbour Set
        self.neigh = [odom for i,odom in enumerate(self.bot_odom) if self.disij[i] <= r and self.disij[i]>0.1 and self.delij[i] <= ca and self.delij[i] >= -ca ]
        self.neigh.append(self.odom)
        # print(len(self.neigh),self.namespace)
        no_neigh = len(self.neigh)

        if no_neigh >= 2:
            self.initial_no = no_neigh
            self.goal.x = np.mean([odom.pose.pose.position.x for odom in self.neigh])
            self.goal.y = np.mean([odom.pose.pose.position.y for odom in self.neigh])
        else:
            if self.goal.x == 3.0 and self.goal.y == -2.0:
                print("called", self.namespace)
                self.goal.x = np.random.uniform(1,6)
                self.goal.y = np.random.uniform(-6,0)
        
    def control(self,k):
        """control law for bot"""
 
        self.set_goal(2,(2*pi/3))        
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

        # Define wall positions
        wall_positions = [(-7.25, self.y), (8.0000, self.y), (self.x < 0.9, -3.65), (self.x, 3.40), (self.x>0.0, 0.65), (self.x, -6.25), (0.0, self.y > 0.5), (0.9, self.y < -3.25)]  # Example wall positions
        # wall_positions = [(-7.25, self.y), (8.0000, self.y), (self.x, -3.65), (self.x, 3.40), (self.x, 0.65), (self.x, -6.25), (0.0, self.y), (0.9, self.y)]
        wall_radius = 0.6

        #Static Obstacles
        obstacle_positions = [(-5.50, 1.50), (-2.20, 1.50), (-4.0,0.0), (-5.50,-1.50), (-2.20,-1.50)] 
        obstacle_radius = 0.6

        # Combine wall positions with other obstacles
        # obstacle_positions += wall_positions
        obstacle_distance = min(np.linalg.norm(np.array([self.x, self.y]) - np.array(obstacle)) for obstacle in obstacle_positions)
        wall_distance = min(np.linalg.norm(np.array([self.x, self.y]) - np.array(wall)) for wall in wall_positions)   
        
        if (self.dis_err) >= 0.85:
            temp = []
            vap = []
            excluded_bot = [self.cur_bot_id_indx]
            # print(self.cur_bot_id_indx)
            for i,z in enumerate(self.disij):
                if i not in excluded_bot:
                    if z >= 0.75 and obstacle_distance > obstacle_radius:
                        self.speed.linear.x = 0.18
                        self.speed.angular.z = K*np.sign(self.dtheta)
                        # print(z,self.delij[i]*(180/pi),i,self.namespace,'1',self.goal, obstacle_distance)
                        print("Free", self.namespace, 'w', wall_distance, 'o', obstacle_distance)
                    elif wall_distance <= wall_radius :
                        self.wall_following()
                        print("Wall Following", obstacle_distance, self.namespace)                    
                    else:                        
                        t = rospy.get_time()                       
                        self.speed.linear.x = max((0.12 -(4000-t)*0.00001),0)                    
                        self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])
                        temp.append(self.speed.angular.z)
                        vap.append(self.speed.linear.x)
                        print("Engaged", self.namespace, obstacle_distance)
                        # print(z,self.delij[i]*(180/pi),i,self.namespace,'2',self.goal, obstacle_distance)
                if temp:
                    self.speed.angular.z = np.mean(temp)
                    self.speed.linear.x = np.mean(vap)   #/len(self.neigh)                
        else:
            if len(self.neigh) < 2 or obstacle_distance < 2.0*obstacle_radius or wall_distance < 2*wall_radius: 
                self.goal = Point(3.0, -2.0, 0.0)
                print("Alone",self.namespace)            
            else:                                            
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                print("Aggreated", self.namespace, obstacle_distance)
                # print(rospy.get_time())
                # print(self.x,self.goal.x,'x')
                # print(self.y,self.goal.y,'y')       
        

        self.cmd_vel.publish(self.speed)
        self.pubg.publish(self.goal)

if __name__ == '__main__':
    k = 0
    l = [] #l is time
    rospy.init_node("Task2_controller")
    rate = rospy.Rate(4)
    bot = robot(6)
    rospy.sleep(10)     

    while not rospy.is_shutdown() and k < 4000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.control(k)            
        rate.sleep()