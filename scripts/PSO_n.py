#!/usr/bin/env python3
import rospy, rospkg, os
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PSOAgent:
    def __init__(self, name):
        self.namespace = name
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.pbest = np.zeros(2)
        self.pbest_val = float('inf')

        self.cmd_pub = rospy.Publisher(f"{name}/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(f"{name}/odom", Odometry, self.odom_callback)
        self.dirname = rospkg.RosPack().get_path('swarm_aggregation')
        self.iters = rospy.get_param("/iteration", 0)
        try:
            os.makedirs(f'{self.dirname}/Data/PSO/Data{self.iters}')
        except FileExistsError:
            pass
        with open('{}/Data/PSO/Data{}/{}.csv'.format(self.dirname,self.iters,self.namespace.split("/")[1]),'a+') as f:
            f.write("time, x, y\n" )

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

    def update_velocity(self, gbest, w=0.5, c1=1.0, c2=1.5):
        r1 = np.random.rand()
        r2 = np.random.rand()

        inertia = w * self.velocity
        cognitive = c1 * r1 * (self.pbest - self.position)
        social = c2 * r2 * (gbest - self.position)

        self.velocity = inertia + cognitive + social
        norm = np.linalg.norm(self.velocity)
        if norm > 0.3:
            self.velocity = (self.velocity / norm) * 0.3  # cap max speed

    def update_position(self):
        self.position += self.velocity

        # Update personal best
        dist = np.linalg.norm(self.position)
        if dist < self.pbest_val:
            self.pbest = self.position.copy()
            self.pbest_val = dist

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.velocity[0]
        cmd.linear.y = self.velocity[1]
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

class SwarmClustering:
    def __init__(self, num_bots):
        self.bots = [PSOAgent(f"/tb3_{i}") for i in range(7)]

    def compute_gbest(self):
        # Using mean of pbest positions as gbest
        pbests = [bot.pbest for bot in self.bots]
        return np.mean(pbests, axis=0)

    def run(self):
        
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            gbest = self.compute_gbest()
            for bot in self.bots:
                bot.update_velocity(gbest)
                bot.update_position()
                bot.publish_cmd()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pso_swarm_clustering")
    swarm = SwarmClustering(num_bots=6)  # Adjust as per your simulation
    swarm.run()