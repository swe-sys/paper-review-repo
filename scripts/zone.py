#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
from sensor_msgs.msg import LaserScan
from swarm_aggregation.msg import botPose
import rospkg
import os

class Robot(object):
    def __init__(self, no_of_bots):
        self.x = 0
        self.y = 0
        self.cur_bot_id_indx = 0
        self.total_bots = no_of_bots
        self.speed = Twist()
        self.ranges = []
        self.bot_odom = [Odometry() for _ in range(self.total_bots)]
        self.neigh = []
        self.goal = Point(np.random.uniform(-10,10), np.random.uniform(-10,10), 0.0)
        self.x_bounds = (-10, 10)
        self.y_bounds = (-10, 10)
        self.detected_lattices = []
        self.known_obstacles = []
        self.lattice_spacing = 1.3

        self.namespace = rospy.get_namespace()
        self.node_name = rospy.get_name()
        self.odom = Odometry()

        self.botpose_sub = rospy.Subscriber('/obs_data', botPose, self.detect_bots)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal', Point, queue_size=10)

    def update_odom(self, msg):
        self.odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.yaw = yaw % (2 * pi)

    def detect_bots(self,data):
        """ Odometry of all bots"""
        bot_id = data.bot_id
        odoms = data.botpose
        self.bot_odom = odoms
        self.cur_bot_id_indx = bot_id.index(self.namespace)

    def obstacle_detector(self, msg):
        self.ranges = msg.ranges

    def is_obstacle_nearby(self, threshold=0.7):
        return any(distance < threshold for distance in self.ranges if distance > 0)

    def wall_following(self):
        deg, dst = 50, 0.7
        if min(self.ranges[:deg]) <= dst or min(self.ranges[-deg:]) <= dst:
            self.speed.angular.z = -0.2
            self.speed.linear.x = 0.0                   
        elif min(self.ranges[deg:120]) < dst:
            self.speed.angular.z = 0.0
            self.speed.linear.x = 0.2

    def distance(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def detect_new_obstacle(self, obs_pos):
        self.known_obstacles.append(obs_pos)
        for prev in self.known_obstacles[:-1]:
            if abs(self.distance(obs_pos, prev) - self.lattice_spacing) < 0.2:
                mid = ((obs_pos[0] + prev[0]) / 2, (obs_pos[1] + prev[1]) / 2)
                dx, dy = obs_pos[0] - prev[0], obs_pos[1] - prev[1]
                d = sqrt(dx**2 + dy**2)
                nx, ny = -dy/d, dx/d
                height = self.lattice_spacing * sqrt(3)/2
                third1 = (mid[0] + nx * height, mid[1] + ny * height)
                third2 = (mid[0] - nx * height, mid[1] - ny * height)
                for pt in self.known_obstacles:
                    if self.distance(pt, third1) < 0.2 or self.distance(pt, third2) < 0.2:
                        self.detected_lattices.append([obs_pos, prev, pt])
                        break

    def is_point_near_lattice(self, point, threshold=1.0):
        for tri in self.detected_lattices:
            if any(self.distance(point, vertex) < threshold for vertex in tri):
                return True
        return False

    def set_goal(self):
        disij = [self.distance((odom.pose.pose.position.x, odom.pose.pose.position.y), (self.x, self.y))
                 for odom in self.bot_odom]
        delij = [atan2((odom.pose.pose.position.y - self.y), (odom.pose.pose.position.x - self.x))
                 for odom in self.bot_odom]
        self.neigh = [odom for i, odom in enumerate(self.bot_odom)
                      if disij[i] <= 2 and disij[i] > 0.1 and -2.1 <= delij[i] <= 2.1]
        self.neigh.append(self.odom)

        if len(self.neigh) >= 2:            
            self.goal.x = np.mean([odom.pose.pose.position.x for odom in self.neigh])
            self.goal.y = np.mean([odom.pose.pose.position.y for odom in self.neigh])
        else:
            while True:
                gx = np.random.uniform(self.x_bounds[0], self.x_bounds[1])
                gy = np.random.uniform(self.y_bounds[0], self.y_bounds[1])
                if not self.is_point_near_lattice((gx, gy)):
                    self.goal = Point(gx, gy, 0.0)
                    break

    def control(self):
        self.set_goal()
        incx = self.goal.x - self.x
        incy = self.goal.y - self.y
        self.bearing = atan2(incy, incx)
        self.dis_err = sqrt(incx**2 + incy**2)
        self.dtheta = self.bearing - self.yaw
        K = 0.3        

        if self.dis_err >= 0.885:
            if len(self.disij) == 0:
                self.speed.linear.x = 0.18
                self.speed.angular.z = K * np.sign(self.dtheta)
            elif len(self.neigh) == 0 and (min(self.ranges[0:30]) <= 0.7 or min(self.ranges[330:360]) <= 0.7):
                self.wall_following()
            else:
                for i, z in enumerate(self.disij):
                    if z >= 0.7:
                        self.speed.linear.x = 0.18
                        self.speed.angular.z = K * np.sign(self.dtheta)
                    else:
                        t = rospy.get_time()
                        self.speed.linear.x = max((0.18 - (5000 - t) * 0.0001), 0)
                        self.speed.angular.z = K * np.sign(self.dtheta) - 0.866 * np.sign(self.delij[i])
        else:
            if len(self.neigh) >= 2:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                rospy.loginfo(f"Aggregated: {self.namespace}")
            else:
                self.goal = Point(3.0, -2.0, 0.0)

        self.cmd_vel_pub.publish(self.speed)
        self.goal_pub.publish(self.goal)

if __name__ == '__main__':
    rospy.init_node("swarm_controller")
    rate = rospy.Rate(4)
    bot = Robot(no_of_bots=6)
    rospy.sleep(6)

    while not rospy.is_shutdown():
        bot.control()
        rate.sleep()