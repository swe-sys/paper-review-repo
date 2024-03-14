#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt
import matplotlib.pyplot as plt

class robot():
    def __init__(self,topic):
        self.topic = topic
        self.x = 0
        self.y = 0        
        self.theta = 0
        self.rot_q = 0       
        self.speed = Twist()
        self.sub =  rospy.Subscriber("/odom",Odometry,self.newOdom)
        self.pub =  rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
                    
    def newOdom(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])  
    
if __name__ == '__main__':
	k = 0
	l = [] #l is time	
	bot = robot(1)
	dis_err = []
	u = []
	# i = 0
	bearing = [0]
	rospy.init_node("Onebot_controller")	
	r = rospy.Rate(4)
	goal = Point()
		
	while not rospy.is_shutdown() and k < 4000:
		dtheta = []
		dis_err = 0
		k = k+1
		h = 0.25
		K = 0.55
		l.append((k+1)/10) # Time
		incx = 0
		incy = 0
		dis_err = 0
		print('x :',bot.x,'y :',bot.y)

		# if (bot.y) > 0:
		goal.x = 2
		goal.y = -2
		# else:
		# 	goal.x = -4
		# 	goal.y = -4              

        #Distance Between goal and Bot    
		incx = (goal.x - bot.x )
		incy = (goal.y - bot.y )

        # Bearing of Bots
		bearing.append(atan2(incy,incx))

		# Distance error between goal and robots position
		dis_err = (sqrt(incx**2+incy**2))

		# Gradient of Bearing
		dtheta = (bearing[k] - bearing[k-1])/h

		if abs(dis_err) >= 0.1:
			print(dis_err,'distance error')
			bot.speed.linear.x = 0.22
			bot.speed.angular.z = K*np.sign(dtheta)
			print(bot.speed.angular.z,'Angular')
		else:
			print('Done!!')
			print(dis_err,'distance error')
			bot.speed.linear.x = 0
			bot.speed.angular.z = 0           

		bot.pub.publish(bot.speed)   
		r.sleep()