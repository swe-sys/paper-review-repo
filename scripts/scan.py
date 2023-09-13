#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi, cos, sin
import numpy as np

class Neighbors:
    def __init__(self):
        self.neighbor_set = set()

    def add_neighbor(self, position):
        self.neighbor_set.add(position)

    def get_neighbors(self):
        return self.neighbor_set

def lidar_callback(scan):
    # Process the LIDAR scan data and detect objects
    detected_objects = []
    neighbors = set()
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment

    for i, distance in enumerate(scan.ranges):
        angle = angle_min + i * angle_increment
        x = distance * cos(angle)
        y = distance * sin(angle)
        detected_objects.append({"position": (x, y)})

    # Store positions of objects in the neighbor set
    for obj in detected_objects:
        position = obj["position"]
        neighbors.add_neighbor(position)

def control_robot(goal_x, goal_y):
    # Calculate the control commands to reach the goal position
    kp = 0.5  # Proportional gain
    max_linear_velocity = 0.2  # Maximum linear velocity
    max_angular_velocity = 1.0  # Maximum angular velocity

    current_x = 0.0  # Placeholder for the current robot position (x-coordinate)
    current_y = 0.0  # Placeholder for the current robot position (y-coordinate)

    # Calculate the linear and angular errors
    linear_error = np.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    angular_error = np.arctan2(goal_y - current_y, goal_x - current_x) - 0.0

    # Apply proportional control to calculate the control commands
    linear_velocity = kp * linear_error
    angular_velocity = kp * angular_error

    # Limit the linear and angular velocities
    linear_velocity = np.clip(linear_velocity, 0.0, max_linear_velocity)
    angular_velocity = np.clip(angular_velocity, -max_angular_velocity, max_angular_velocity)

    # Publish the control commands
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    cmd_vel_pub.publish(twist)

def main():
    rospy.init_node('lidar_listener')
    neighbors = Neighbors()

    # Create a subscriber to listen to the LIDAR scan topic
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.spin()

    # Retrieve the stored neighbor positions
    neighbor_positions = neighbors.get_neighbors()

    if len(neighbor_positions) > 0:
        # Calculate the mean of x and y coordinates
        goal_x = np.mean([pos[0] for pos in neighbor_positions])
        goal_y = np.mean([pos[1] for pos in neighbor_positions])
        print("chal ja")

        control_robot(goal_x, goal_y)
    else:
        print("No neighbors detected.")

if __name__ == "__main__":
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    main()
