#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, cos, sin, pi
from swarm_aggregation.msg import botPose
import random

class PSOClusterRobot:
    def __init__(self, robot_id, total_bots):
        self.id = robot_id
        self.total_bots = total_bots
        self.namespace = rospy.get_namespace()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom = Odometry()
        self.bot_odom = [Odometry() for _ in range(self.total_bots)]

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.update_odom)
        rospy.Subscriber("/scan", LaserScan, self.update_lidar)
        rospy.Subscriber("/obs_data", botPose, self.update_bots)

        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        self.velocity = np.array([0.0, 0.0])
        self.personal_best = np.array([0.0, 0.0])

        self.cluster_goals = [
            np.array([random.uniform(-5, 5), random.uniform(-5, 5)]) for _ in range(3)
        ]
        self.assigned_cluster = self.cluster_goals[self.id % len(self.cluster_goals)]

        self.inertia = 0.5
        self.cognitive = 1.5
        self.social = 1.5

    def update_odom(self, msg):
        self.odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        if self.yaw < 0:
            self.yaw += 2 * pi

    def update_lidar(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def update_bots(self, msg):
        self.bot_odom = msg.botpose

    def control(self):
        twist = Twist()

        pos = np.array([self.x, self.y])
        if np.linalg.norm(pos - self.personal_best) < 0.1:
            self.personal_best = pos

        r1 = random.random()
        r2 = random.random()

        cognitive_term = self.cognitive * r1 * (self.personal_best - pos)
        social_term = self.social * r2 * (self.assigned_cluster - pos)

        self.velocity = self.inertia * self.velocity + cognitive_term + social_term

        goal = pos + self.velocity * 0.1
        dx, dy = goal[0] - self.x, goal[1] - self.y
        angle_to_goal = atan2(dy, dx) - self.yaw

        if angle_to_goal > pi:
            angle_to_goal -= 2 * pi
        if angle_to_goal < -pi:
            angle_to_goal += 2 * pi

        if sqrt(dx**2 + dy**2) > 0.1:
            twist.linear.x = 0.15
            twist.angular.z = 0.5 * angle_to_goal
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        rospy.loginfo(f"[{self.namespace}] Assigned Cluster Goal: {self.assigned_cluster.tolist()}, Pos: ({self.x:.2f}, {self.y:.2f})")
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('pso_cluster_controller')
    total_robots = rospy.get_param("~total_robots", 6)
    robot_id = rospy.get_param("~robot_id", 0)
    rate = rospy.Rate(4)
    bot = PSOClusterRobot(robot_id, total_robots)
    rospy.sleep(5)
    while not rospy.is_shutdown():
        bot.control()
        rate.sleep()