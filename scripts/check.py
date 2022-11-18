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
    bx1 = [0]
    bx2 = [0]
    bx3 = [0]
    by1 = [0]
    by2 = [0]
    by3 = [0]
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

    while not rospy.is_shutdown() and k < 400:
        k = k+1
        h = 0.25
        K = 0.55
        
        # Goal Position
        goal.x = 4
        goal.y = 4
        
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
        bx1.append(x1)
        bx2.append(x2)
        bx3.append(x3)
        by1.append(y1)
        by2.append(y2)
        by3.append(y3)

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
        print(dis_err1,dis_err2,dis_err3,'11')
        print(u1[k])
        print(u2[k])
        print(u3[k])
        print(disij,disjk,diski,'12')
        print(dtheta1,dtheta2,dtheta3,'13')
        print(delij,delki,deljk,'14')
        
        # Checking all the posiblities of collision
        if (abs(dis_err1) >= 0.75 or abs(dis_err2) >= 0.75 or abs(dis_err3) >= 0.75):

            if (abs(disij) > 0.25 or abs(delij) >= pi/2):
                print('1')                
                speed1.linear.x = 0.18
                speed1.angular.z = u1[k]
                speed2.linear.x = 0.18
                speed2.angular.z = u3[k]
            else:
                print('2')                
                t = rospy.get_time()
                print(t)
                speed1.linear.x = 0.01
                speed1.angular.z = u4[k]
                speed2.linear.x = 0.01
                speed2.angular.z = u4[k]

            if (abs(disjk) > 0.25 or abs(deljk) >= pi/2):
                print('3')
                speed2.linear.x = 0.18
                speed2.angular.z = u2[k]
                speed3.linear.x = 0.18
                speed3.angular.z = u3[k]
            else:
                print('4')                
                t = rospy.get_time()
                print(t)
                speed2.linear.x = 0.01
                speed2.angular.z = u5[k]
                speed3.linear.x = 0.01
                speed3.angular.z = u5[k]

            if (abs(diski) > 0.25 or abs(delki) >= pi/2):
                print('5')               
                speed1.linear.x = 0.18
                speed1.angular.z = u1[k]
                speed3.linear.x = 0.18
                speed3.angular.z = u3[k]
            else:
                print('6')                
                t = rospy.get_time()
                print(t)
                speed3.linear.x = 0.01
                speed3.angular.z = u6[k]
                speed1.linear.x = 0.01
                speed1.angular.z = u6[k]
                          
        else:
            print('Done')
            t = rospy.get_time()
            print(t)
            speed1.linear.x = 0.0
            speed1.angular.z = 0.0
            speed2.linear.x = 0.0
            speed2.angular.z = 0.0 
            speed3.linear.x = 0.0
            speed3.angular.z = 0.0         
                
        pub1.publish(speed1)
        pub2.publish(speed2)
        pub3.publish(speed3)
        r.sleep()

# Plotting
plt.figure(1)
plt.plot(l,u1 , label='Free')
plt.plot(l,u4 , label='Engaged')
plt.xlabel('Time')
plt.ylabel('Control law')
plt.legend()
plt.show()


plt.figure(2)
plt.plot(l,u2 , label='Free')
plt.plot(l,u5, label='Engaged')
plt.xlabel('Time')
plt.ylabel('Control law')
plt.legend()
plt.show()

plt.figure(3)
plt.plot(l,u3 , label='Free')
plt.plot(l,u6 , label='Engaged')
plt.xlabel('Time')
plt.ylabel('Control law')
plt.legend()
plt.show()

plt.figure(4)
plt.plot(l,bx1, label='Bot1')
plt.plot(l,bx2, label='Bot2')
plt.plot(l,bx3, label='Bot3')
plt.plot(l,s1, label='Goal')
plt.xlabel('Time')
plt.ylabel('X Coordinate')
plt.legend()
plt.show()

plt.figure(5)
plt.plot(l,by1, label='Bot1')
plt.plot(l,by2, label='Bot2')
plt.plot(l,by3, label='Bot3')
plt.plot(l,s2, label='Goal')
plt.xlabel('Time')
plt.ylabel('Y Coordinate')
plt.legend()
plt.show()

plt.figure(6)
plt.plot(bx1,by1, label='Bot1')
plt.plot(bx2,by2, label='Bot2')
plt.plot(bx3,by3, label='Bot3')
plt.plot(l,s2, label='Goal')
plt.xlabel('Time')
plt.ylabel('Y Coordinate')
plt.legend()
plt.show()

