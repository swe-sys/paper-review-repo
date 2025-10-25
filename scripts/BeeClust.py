#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, cos, sin, pi
from swarm_aggregation.msg import botPose
import random, os, rospkg

class BeeclustRobot:
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

        self.stop_timer = 0
        self.move_timer = random.randint(20, 50)
        self.turn_timer = 0
        self.rotation_direction = random.choice([-1, 1])

        self.stop_duration = 50
        self.temperature_field = lambda x, y: np.exp(-((x**2 + y**2)/10.0))

        self.arena_center = np.array([0.0, 0.0])
        self.currently_stopping = False
        self.received_odom = False

        self.no_neighbor_counter = 0
        self.no_neighbor_threshold = 200

        self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        self.iters = rospy.get_param("/iteration", 0)
        try:
            os.makedirs(f'{self.dirname}/Data/BeeClust/Data{self.iters}')
        except FileExistsError:
            pass
        with open('{}/Data/BeeClust/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("time, x, y\n" )        

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
        self.received_odom = True

    def update_lidar(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def update_bots(self, msg):
        self.bot_odom = msg.botpose

    def get_closest_visible_neighbor_distance(self):
        min_dist = float('inf')
        if not self.ranges:
            return min_dist
        for i, odom in enumerate(self.bot_odom):
            if i == self.id:
                continue
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
            if abs(angle) <= pi / 3 and 0.1 < dist < 3:
                lidar_idx = int((angle - self.angle_min) / self.angle_increment)
                if 0 <= lidar_idx < len(self.ranges):
                    if self.ranges[lidar_idx] > dist - 0.1:
                        min_dist = min(min_dist, dist)
        rospy.loginfo(f"[{self.namespace}] Closest visible neighbor distance: {min_dist if min_dist < float('inf') else 'None'}")
        return min_dist

    def stop_based_on_temperature(self):
        temp = self.temperature_field(self.x, self.y)
        self.stop_timer = int(self.stop_duration * temp)
        self.currently_stopping = True
        rospy.loginfo(f"[{self.namespace}] Stopping due to neighbor. Temp: {temp:.2f}, Stop timer: {self.stop_timer}")

    def control(self):
        with open('{}/Data/BeeClust/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("{},{},{}".format(rospy.get_time(),self.x, self.y) + '\n')
        
        if not self.received_odom:
            rospy.logwarn(f"[{self.namespace}] Waiting for odometry...")
            return

        twist = Twist()
        closest_neighbor_dist = self.get_closest_visible_neighbor_distance()

        if closest_neighbor_dist < 0.85 and not self.currently_stopping:
            rospy.loginfo(f"[{self.namespace}] Neighbor detected within 0.5m, initiating stop.")
            self.stop_based_on_temperature()
            self.no_neighbor_counter = 0
        elif closest_neighbor_dist >= float('inf'):
            self.no_neighbor_counter += 1
        else:
            self.no_neighbor_counter = 0

        if self.stop_timer > 0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.stop_timer -= 1
            rospy.loginfo(f"[{self.namespace}] Stopped. Timer: {self.stop_timer}")
            if self.stop_timer == 0:
                self.currently_stopping = False

        elif self.no_neighbor_counter > self.no_neighbor_threshold:
            dx = self.arena_center[0] - self.x
            dy = self.arena_center[1] - self.y
            angle = atan2(dy, dx) - self.yaw
            if angle > pi:
                angle -= 2 * pi
            elif angle < -pi:
                angle += 2 * pi
            twist.linear.x = 0.15
            twist.angular.z = 0.5 * angle
            rospy.loginfo(f"[{self.namespace}] No neighbor for long, seeking center")

        elif self.move_timer > 0:
            twist.linear.x = 0.15
            twist.angular.z = 0.0
            self.move_timer -= 1
            rospy.loginfo(f"[{self.namespace}] Moving forward. Timer: {self.move_timer}")

        elif self.turn_timer > 0:
            twist.linear.x = 0.0
            twist.angular.z = 0.5 * self.rotation_direction
            self.turn_timer -= 1
            rospy.loginfo(f"[{self.namespace}] Turning. Timer: {self.turn_timer}")

        else:
            self.turn_timer = random.randint(5, 20)
            self.move_timer = random.randint(20, 50)
            self.rotation_direction = random.choice([-1, 1])
            rospy.loginfo(f"[{self.namespace}] New turn/move cycle started.")

        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('beeclust_controller')
    total_robots = rospy.get_param("~total_robots", 6)
    robot_id = rospy.get_param("~robot_id", 0)
    rate = rospy.Rate(4)
    bot = BeeclustRobot(robot_id, total_robots)
    rospy.sleep(6)
    while not rospy.is_shutdown():
        bot.control()
        rate.sleep()