#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, cos, sin, pi
from swarm_aggregation.msg import botPose

class BoidsRobot:
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

        self.min_x, self.max_x = -7.0, 7.0
        self.min_y, self.max_y = -5.0, 5.0

        self.stop_threshold = 0.4  # stop if neighbor is closer than this

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

    def get_neighbors(self, view_distance=3, view_angle=pi/3):
        neighbors = []
        for i, odom in enumerate(self.bot_odom):
            # if i == self.id:
            #     continue
            px = odom.pose.pose.position.x
            py = odom.pose.pose.position.y
            dx = px - self.x
            dy = py - self.y
            dist = sqrt(dx ** 2 + dy ** 2)
            angle = atan2(dy, dx) - self.yaw
            if angle < -pi:
                angle += 2 * pi
            if angle > pi:
                angle -= 2 * pi
            if abs(angle) <= view_angle and 0.1 < dist < view_distance:
                neighbors.append((px, py, dist))
        return neighbors

    def boids_control(self):
        neighbors = self.get_neighbors()
        twist = Twist()

        # rospy.loginfo(f"[{self.namespace}] Current Pos: ({self.x:.2f}, {self.y:.2f}), Yaw: {self.yaw:.2f}, Neighbors: {len(neighbors)}")

        # Boundary constraint force
        bx = by = 0.0
        margin = 0.5
        gain = 1.0
        if self.x < self.min_x + margin:
            bx += gain
        if self.x > self.max_x - margin:
            bx -= gain
        if self.y < self.min_y + margin:
            by += gain
        if self.y > self.max_y - margin:
            by -= gain

        # Stop if any neighbor is too close
        # for _, _, dist in neighbors:
        #     if dist < self.stop_threshold:
        #         rospy.loginfo(f"[{self.namespace}] Stopping due to close neighbor at distance {dist:.2f}")
        #         twist.linear.x = 0.0
        #         twist.angular.z = 0.0
        #         self.cmd_vel_pub.publish(twist)
        #         return

        if not neighbors:
            dx = bx
            dy = by
            if dx == 0 and dy == 0:
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                rospy.loginfo(f"[{self.namespace}] No neighbors - moving forward.")
            else:
                angle = atan2(dy, dx) - self.yaw
                if angle > pi:
                    angle -= 2 * pi
                if angle < -pi:
                    angle += 2 * pi
                twist.linear.x = 0.15
                twist.angular.z = 0.5 * angle
                rospy.loginfo(f"[{self.namespace}] No neighbors - correcting boundary direction.")
        else:
            positions = np.array([(px, py) for px, py, _ in neighbors])
            centroid = np.mean(positions, axis=0)
            dx = centroid[0] - self.x + bx
            dy = centroid[1] - self.y + by
            angle_to_centroid = atan2(dy, dx) - self.yaw
            distance = sqrt((self.x-centroid[0])**2 + (self.y -centroid[1])**2)            
            if distance > 0.5:
                if angle_to_centroid > pi:
                    angle_to_centroid -= 2 * pi
                if angle_to_centroid < -pi:
                    angle_to_centroid += 2 * pi
                twist.linear.x = 0.15
                twist.angular.z = 0.5 * angle_to_centroid
                rospy.loginfo(f"[{self.namespace}] Steering to centroid ({centroid[0]:.2f}, {centroid[1]:.2f})")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("stopped",self.namespace)            

        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('boids_controller')
    total_robots = rospy.get_param("~total_robots", 6)
    robot_id = rospy.get_param("~robot_id", 0)
    rate = rospy.Rate(4)
    bot = BoidsRobot(robot_id, total_robots)
    rospy.sleep(5)
    while not rospy.is_shutdown():
        bot.boids_control()
        rate.sleep()