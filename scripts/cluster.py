#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
#from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import atan2, pi, sqrt
import matplotlib.pyplot as plt


class robot():
    def __init__(self,topic):
        self.topic = topic
        self.x = 0
        self.y = 0        
        self.theta = 0
        self.rot_q = 0
        self.xm = []
        self.ym = []
        self.speed = Twist()
        self.sub =  rospy.Subscriber(self.topic+"/odom",Odometry,self.newOdom)
        #self.model_states = rospy.Subscriber("/gazebo/model_states",ModelStates,self.newModel)
        self.pub =  rospy.Publisher(self.topic+"/cmd_vel", Twist, queue_size = 1)
                    
    def newOdom(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])  

    
if __name__ == '__main__':
    k = 0 
    i = 0
    j = 0
    l= [0] 
    bearing = []       
    rospy.init_node("Clustering")
    bot = [robot("/tb3_%d" %i) for i in range(5)]
    r = rospy.Rate(40)
    goal = Point()

    while not rospy.is_shutdown() and k < 4000:
        dis = []
        dell = []
        u = []
        v=[] 
        dtheta = []
        k = k+1
        h = 0.25
        K = 0.55
        l.append((k+1)/10) # Time
        incx = []
        incy = []
        angle_to_goal = []
        dis_err = []

        for i in range(5):
            if (bot[i].y) >= 0:
                goal.x = 4
                goal.y = 4
            else:
                goal.x = -4
                goal.y = -4              

            #Distance Between goal and Bot    
            incx.append(goal.x - bot[i].x)
            incy.append(goal.y - bot[i].y)

            # Bearing of Bots
            bearing.append(atan2(incy[i],incx[i]))   

        for i in range(5):
            dell.append([bot[i].theta + bearing[j] for i in range(5)])
            dis.append([sqrt((bot[i].y - bot[j].y)**2 + (bot[i].x - bot[j].x)**2) for j in range(5)]) 
            dis_err.append([(incx[i]**2+incy[i]**2)**0.5 for i in range(5)])
            dtheta.append((bearing[k] - bearing[k-1])/h)
            print(i)
            print(dis_err[i],'11')
            print(dis[i],'12')
            print(dtheta[i],'13')
            for j in range(5):
                if (i!= j):
                    #Control law 
                    v.append(K*np.sign(dtheta[i]))    # Free Subsystem            
                    u.append(v[i] - 1.5*np.sign(dell[i][j]))  # Clustered Subsystem                  
            
                    if abs(dis_err[i]) > 0.5 :
                        print(dis_err[i],'1')
                        print(u[i],'2')
                        print(v[i],'3')
                        bot[i].speed.linear.x = 0.22
                        bot[i].speed.angular.z = v[i]
                        #if dell[i][j] <= pi/9 or abs(dis[i][j])< 2:
                            #print('11')
                            #bot[i].speed.linear.x = 0.10
                            #bot[i].speed.angular.z = u[i]
                    else:
                        print("2")
                        bot[i].speed.linear.x = 0.0
                        bot[i].speed.angular.z = 0.0
            

            bot[i].pub.publish(bot[i].speed)   
        r.sleep()

# Plotting
# plt.figure(1)
# plt.plot(l,u[1] , label='bot1')
# plt.plot(l,u[2] , label='bot2')
# plt.plot(l,u[3] , label='bot3')
# plt.plot(l,u[4] , label='bot4')
# plt.plot(l,u[5] , label='bot5')
# plt.xlabel('Time')
# plt.ylabel('Cluster Control law')
# plt.legend()
# plt.show()


# plt.figure(2)
# plt.plot(l,v[1] , label='bot1')
# plt.plot(l,v[2] , label='bot2')
# plt.plot(l,v[3] , label='bot3')
# plt.plot(l,v[4] , label='bot4')
# plt.plot(l,v[5] , label='bot5')
# plt.xlabel('Time')
# plt.ylabel('Free Control law')
# plt.legend()
# plt.show()