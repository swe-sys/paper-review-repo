#!/usr/bin/env python3

from math import sqrt
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from swarm_aggregation.msg import botPose, bot
from tf.transformations import euler_from_quaternion

class Obstacles:

    def __init__(self):

        self.obs_pub = rospy.Publisher('/obs_data', botPose, queue_size = 10)
        #uncomment following lines if using gazebo simulation
        rospy.Subscriber('/tb3_0/odom', Odometry, self.callback_odom, '/tb3_0/')
        rospy.Subscriber('/tb3_1/odom', Odometry, self.callback_odom, '/tb3_1/')
        rospy.Subscriber('/tb3_2/odom', Odometry, self.callback_odom, '/tb3_2/')
        rospy.Subscriber('/tb3_3/odom', Odometry, self.callback_odom, '/tb3_3/')
        rospy.Subscriber('/tb3_4/odom', Odometry, self.callback_odom, '/tb3_4/')
        rospy.Subscriber('/tb3_5/odom', Odometry, self.callback_odom, '/tb3_5/')
        self.obs = {}
        self.obs['/tb3_0/'] = Odometry()
        self.obs['/tb3_1/'] = Odometry()
        self.obs['/tb3_2/'] = Odometry()
        self.obs['/tb3_3/'] = Odometry()
        self.obs['/tb3_4/'] = Odometry()
        self.obs['/tb3_5/'] = Odometry()

        #comment following lines if trying to gazebo simulation
        # self.prev_pose = {}
        # self.prev_pose['/tb3_0/'] = Pose2D()
        # self.prev_pose['/tb3_1/'] = Pose2D()
        # self.prev_pose['/tb3_2/'] = Pose2D()
        # self.prev_pose['/tb3_3/'] = Pose2D()
        # self.obs = {}
        # self.obs['/tb3_0/'] = Odometry()
        # self.obs['/tb3_1/'] = Odometry()
        # self.obs['/tb3_2/'] = Odometry()
        # self.obs['/tb3_3/'] = Odometry()
        # rospy.Subscriber('/vicon/fb5_10/fb5_10', TransformStamped, self.callback_odom, '/tb3_1/')
        # rospy.Subscriber('/vicon/fb5_13/fb5_13', TransformStamped, self.callback_odom, '/tb3_2/')
        # rospy.Subscriber('/vicon/fb5_12/fb5_12', TransformStamped, self.callback_odom, '/tb3_0/')
        # rospy.Subscriber('/vicon/fb5_2/fb5_2', TransformStamped, self.callback_odom, '/tb3_3/')

    def callback_odom(self, data, bot_id):
        # odom = Odometry()
        # odom.pose.pose.position.x = data.transform.translation.x
        # odom.pose.pose.position.y = data.transform.translation.y
        # odom.pose.pose.position.z = data.transform.translation.z

        # odom.twist.twist.linear.x = sqrt((data.transform.translation.x - self.prev_pose[bot_id].x)**2 + (data.transform.translation.y -self.prev_pose[bot_id].y)**2)/0.01
        # odom.twist.twist.linear.y = 0
        # (roll, pitch, yaw) = euler_from_quaternion([data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w])
        # if yaw < 0:
        #     yaw += 2 * math.pi
        # odom.twist.twist.angular.z = (yaw - self.prev_pose[bot_id].theta)/0.01
        
        
        # self.obs[bot_id] = odom
        # self.prev_pose[bot_id].x = data.transform.translation.x
        # self.prev_pose[bot_id].y = data.transform.translation.y
        
        # self.prev_pose[bot_id].theta = yaw

        # for simulation uncoment below
        self.obs[bot_id] = data

if __name__ == '__main__':
    rospy.init_node('collision_cone_obstacles', anonymous = True)
    s = Obstacles()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        obs = []
        bot_id = []
        for i, n in s.obs.items():
            obs.append(n)
            bot_id.append(i)

        obs_data = botPose()
        obs_data.botpose = obs
        obs_data.bot_id = bot_id
        # print(obs_data)
        s.obs_pub.publish(obs_data)
        # r.sleep() 
