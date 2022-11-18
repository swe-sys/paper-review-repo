#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, pi
import matplotlib.pyplot as plt

x1 = 0.0
y1 = 0.0
theta1 = 0.0

x2 = 0.0
y2 = 2.0
theta2 = 0.0

x3 = 2.0
y3 = 0.0
theta3 = 0.0

def newOdom1(msg):
    global x1
    global y1
    global theta1
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll1, pitch1, theta1) = euler_from_quaternion([rot_q.x , rot_q.y , rot_q.z , rot_q.w ])

def newOdom2(msg):
    global x2
    global y2
    global theta2
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll2, pitch2, theta2) = euler_from_quaternion([rot_q.x , rot_q.y , rot_q.z , rot_q.w ])

def newOdom3(msg):
    global x3
    global y3
    global theta3
    x3 = msg.pose.pose.position.x
    y3 = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll3, pitch3, theta3) = euler_from_quaternion([rot_q.x , rot_q.y , rot_q.z , rot_q.w ])

if __name__ == '__main__':
    k = 0
    l= [0]
    s1 = [0]
    s2 = [0]
    bearing1 = [0]
    bearing2 = [0]
    bearing3 = [0]
    u1 = [0]
    u2 = [0]
    u3 = [0]
    u4 = [0]
    u5 = [0]
    u6 = [0]
    delij = [0]
    delki = [0]
    deljk = [0]
    dis_err = [0]
    rospy.init_node("speed_controller")
    sub1 = rospy.Subscriber("/tb3_0/odom", Odometry, newOdom1)
    pub1 = rospy.Publisher("/tb3_0/cmd_vel", Twist, queue_size = 1)
    sub2 = rospy.Subscriber("/tb3_1/odom", Odometry, newOdom2)
    pub2 = rospy.Publisher("/tb3_1/cmd_vel", Twist, queue_size = 1)
    sub3 = rospy.Subscriber("/tb3_2/odom", Odometry, newOdom3)
    pub3 = rospy.Publisher("/tb3_2/cmd_vel", Twist, queue_size = 1)
    speed1 = Twist()
    speed2 = Twist()
    speed3 = Twist()
    r = rospy.Rate(4)
    goal = Point()

    while not rospy.is_shutdown() and k < 500:
        k = k+1
        h = 0.25
        K = 0.55
        
        # Goal Position
        goal.x = 10
        goal.y = 10
        
        # Distance between goal and robot pose
        inc_x1 = goal.x -x1
        inc_y1 = goal.y -y1
        inc_x2 = goal.x -x2
        inc_y2 = goal.y -y2
        inc_x3 = goal.x -x3
        inc_y3 = goal.y -y3

        l.append((k+1)/10) # Time       
        s1.append(goal.x)
        s2.append(goal.y)

        # Bearing of bots
        angle_to_goal1 = atan2(inc_y1, inc_x1)
        angle_to_goal2  = atan2(inc_y2,inc_x2)
        angle_to_goal3  = atan2(inc_y3,inc_x3)
        bearing1.append(angle_to_goal1)
        bearing2.append(angle_to_goal2)
        bearing3.append(angle_to_goal3)

        # Distance between the robots
        disij = ((x1-x2)**2 + (y1-y2)**2)**0.5
        disjk = ((x2-x3)**2 + (y2-y3)**2)**0.5
        diski = ((x3-x1)**2 + (y3-y1)**2)**0.5

        # Distance error between goal and robots position
        dis_err1 = (inc_x1**2+inc_y1**2)**0.5
        dis_err2 = (inc_x2**2+inc_y2**2)**0.5
        dis_err3 = (inc_x3**2+inc_y3**2)**0.5

        # Gradient of the bearing 
        dtheta1 = (bearing1[k] - bearing1[k-1])/h
        dtheta2 = (bearing2[k] - bearing2[k-1])/h
        dtheta3 = (bearing3[k] - bearing3[k-1])/h

        # bearing of j wrt i
        delij = theta1 + bearing2[k]
        # bearing of k wrt i
        delki = theta3 + bearing1[k]
        # bearing of k wrt j
        deljk = theta2 + bearing3[k]

        # Control law
        u1.append(K*(np.sign(dtheta1)))
        u2.append(K*(np.sign(dtheta2)))
        u3.append(K*(np.sign(dtheta3)))
        u4.append(u1[k] - 1.5*np.sign(delij))
        u5.append(u2[k] - 1.5*np.sign(deljk))
        u6.append(u3[k] - 1.5*np.sign(delki))
        
        # Checking all the posiblities of collision
        if (dis_err1 >= 0.5):

            if (disij > 0.25 or delij >= pi/2):
                print('1')
                print(delij)
                print(dtheta1)
                print(dtheta2)
                speed1.linear.x = 0.22
                speed1.angular.z = u1[k]
                speed2.linear.x = 0.22
                speed2.angular.z = u3[k]
            else:
                print('2')
                print(delij)
                print(dtheta1)
                print(dtheta2)
                speed1.linear.x = 0.01
                speed1.angular.z = u4[k]
                speed2.linear.x = 0.01
                speed2.angular.z = u4[k]

            if (diski > 0.25 or delki >= pi/2):
                print('3')
                print(delki)
                print(dtheta1)
                print(dtheta2)
                speed1.linear.x = 0.22
                speed1.angular.z = u1[k]
                speed3.linear.x = 0.22
                speed3.angular.z = u3[k]
            else:
                print('4')
                print(delki)
                print(dtheta1)
                print(dtheta2)
                speed3.linear.x = 0.01
                speed3.angular.z = u6[k]
                speed1.linear.x = 0.01
                speed1.angular.z = u6[k]               
        else:
            print('1 ruka')
            speed1.linear.x = 0.0
            speed1.angular.z = 0.0

        if (dis_err3 >= 0.5):

            if (disjk > 0.25 or deljk >= pi/2):
                print('5')
                print(deljk)
                print(dtheta1)
                print(dtheta2)
                speed2.linear.x = 0.22
                speed2.angular.z = u2[k]
                speed3.linear.x = 0.22
                speed3.angular.z = u3[k]
            else:
                print('6')
                print(deljk)
                print(dtheta1)
                print(dtheta2)
                speed2.linear.x = 0.01
                speed2.angular.z = u5[k]
                speed3.linear.x = 0.01
                speed3.angular.z = u5[k]

            if (diski > 0.25 or delki >= pi/2):
                print('7')
                print(delki)
                print(dtheta1)
                print(dtheta2)
                speed1.linear.x = 0.22
                speed1.angular.z = u1[k]
                speed3.linear.x = 0.22
                speed3.angular.z = u3[k]
            else:
                print('8')
                print(delki)
                print(dtheta1)
                print(dtheta2)
                speed3.linear.x = 0.01
                speed3.angular.z = u6[k]
                speed1.linear.x = 0.01
                speed1.angular.z = u6[k]               
        else:
            print('3 ruka')
            speed3.linear.x = 0.0
            speed3.angular.z = 0.0
        
        if (dis_err2 >= 0.5):

            if (disij > 0.25 or delij >= pi/2):
                print('9')
                print(delij)
                print(dtheta1)
                print(dtheta2)
                speed1.linear.x = 0.22
                speed1.angular.z = u1[k]
                speed2.linear.x = 0.22
                speed2.angular.z = u2[k]
            else:
                print('10')
                print(delij)
                print(dtheta1)
                print(dtheta2)
                speed1.linear.x = 0.01
                speed1.angular.z = u4[k]
                speed2.linear.x = 0.01
                speed2.angular.z = u4[k]

            if (disjk > 0.25 or deljk >= pi/2):
                print('11')
                print(deljk)
                print(dtheta1)
                print(dtheta2)
                speed2.linear.x = 0.22
                speed2.angular.z = u2[k]
                speed3.linear.x = 0.22
                speed3.angular.z = u3[k]
            else:
                print('12')
                print(deljk)
                print(dtheta1)
                print(dtheta2)
                speed2.linear.x = 0.01
                speed2.angular.z = u5[k]
                speed3.linear.x = 0.01
                speed3.angular.z = u5[k]
        else:
            print('2 ruka')
            speed2.linear.x = 0.0
            speed2.angular.z = 0.0
                
        pub1.publish(speed1)
        pub2.publish(speed2)
        pub3.publish(speed3)
        r.sleep()

# Plotting
plt.figure(1)
plt.plot(l,u1 , label='u1')
plt.plot(l,u2 , label='u2')
plt.plot(l,u3 , label='u3')
plt.plot(l,u4 , label='u4')
plt.plot(l,u5 , label='u5')
plt.xlabel('Time')
plt.ylabel('Control law')
plt.legend()
plt.show()
