#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math, rospkg, os

class KMeansRobot:
    def __init__(self, robot_id, total_robots, n_clusters=2):
        self.id = robot_id
        self.ns = f"/tb3_{robot_id}"
        self.total_robots = total_robots
        self.n_clusters = n_clusters
        self.namespace = rospy.get_namespace()

        # State
        self.position = np.zeros(2)
        self.cluster_id = None

        # Communication arrays
        self.robot_positions = np.zeros((self.total_robots, 2))
        self.centroids = np.random.uniform(-2.0, 2.0, (self.n_clusters, 2))

        # ROS setup
        rospy.Subscriber(f"{self.ns}/odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher(f"{self.ns}/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        self.iters = rospy.get_param("/iteration",0)
        try:
            os.makedirs(f'{self.dirname}/Data/Kmeans/Data{self.iters}')
        except FileExistsError:
            pass
        with open('{}/Data/Kmeans/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("time, goal_x, goal_y, x, y\n" )
        rospy.sleep(1)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.position = np.array([pos.x, pos.y])

    def move_towards(self, target):
        """Moves the robot towards the given 2D target centroid."""
        twist = Twist()
        dx, dy = target[0] - self.position[0], target[1] - self.position[1]
        dist = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy, dx)
        
        if dist > 0.05:
            twist.linear.x = min(0.2, 0.1 + 0.3 * dist)
            twist.angular.z = 0.5 * theta
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        self.vel_pub.publish(twist)

    def assign_clusters(self):
        """Assign each robot to nearest centroid."""
        dists = np.linalg.norm(self.robot_positions - self.centroids[:, np.newaxis], axis=2)
        assignments = np.argmin(dists, axis=0)
        return assignments

    def update_centroids(self, assignments):
        """Update cluster centroids based on assigned robots."""
        new_centroids = np.zeros_like(self.centroids)
        for k in range(self.n_clusters):
            members = self.robot_positions[assignments == k]
            if len(members) > 0:
                new_centroids[k] = np.mean(members, axis=0)
            else:
                new_centroids[k] = self.centroids[k]  # unchanged if empty
        return new_centroids

    def run(self):
        """Main K-means control loop."""
        rospy.sleep(1)        
        prev_centroids = np.copy(self.centroids)

        while not rospy.is_shutdown():
            # In a real ROS setup, these positions would be shared via topics or rosparam
            for i in range(self.total_robots):
                try:
                    pos = rospy.get_param(f"/tb3_{i+1}/position")
                    self.robot_positions[i] = np.array(pos)
                except KeyError:
                    pass

            assignments = self.assign_clusters()
            self.cluster_id = assignments[self.id - 1]
            new_centroids = self.update_centroids(assignments)

            # Move toward assigned cluster centroid
            target = new_centroids[self.cluster_id]
            with open('{}/Data/Kmeans/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
                f.write("{},{},{},{},{}".format(rospy.get_time(),target[0],target[1],self.position[0],self.position[1]) + '\n')

            
            self.move_towards(target)

            # Check convergence
            centroid_shift = np.linalg.norm(new_centroids - prev_centroids)
            if centroid_shift < 0.05:
                rospy.loginfo(f"K-means converged for robot {self.id}")
                self.stop()
                break

            prev_centroids = np.copy(new_centroids)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("kmeans_robot_controller")
    robot_id = rospy.get_param("~robot_id", 1)
    total_robots = rospy.get_param("~total_robots", 6)
    k = rospy.get_param("~n_clusters", 2)

    bot = KMeansRobot(robot_id, total_robots, k)
    bot.run()