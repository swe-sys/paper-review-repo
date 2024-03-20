#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, sqrt, pi, cos, sin,inf,isinf
from collections import deque
import math as m
import os
import rospkg
from std_srvs.srv import Empty

def callback_step(data,bot_id):
    x = data.linear.x
    w = data.angular.z
    if x==0.0 and w==0.0:
        converged[bot_id]=True
    else:
        converged[bot_id] = False
        
if __name__ == '__main__':
    rospy.init_node("obstacle_controller")
    rospy.loginfo("Chal Gye badde")
    count = 1
    dirname = rospkg.RosPack().get_path('swarm_aggregation')
    rate = rospy.Rate(10)
    total_bots = 6
    converged = np.zeros(total_bots).astype(bool)
    cmd_vels = [rospy.Subscriber(f"/tb3_{i+1}/cmd_vel",Twist,callback_step,i) for i in range(total_bots)]
    rospy.sleep(6)
    # stop_server = rospy.ServiceProxy('/gazebo/reset_world',Empty)    
    while not rospy.is_shutdown():
        # print(converged)
        if converged.all():
            os.system('notify-send " {} SINNED {}"'.format(rospy.get_rostime().secs,rospy.get_rostime()))
            with open(f'{dirname}/scripts/time.csv',"a+") as f:
                f.write(f"{count}, {total_bots}, {rospy.get_rostime()},{rospy.get_rostime().secs},{rospy.get_rostime().nsecs}\n")           
            break 
        rate.sleep()