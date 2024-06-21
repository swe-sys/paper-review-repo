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
        # rospy.Subscriber('/tb3_0/odom', Odometry, self.callback_odom, '/tb3_0/')
        rospy.Subscriber('/tb3_1/odom', Odometry, self.callback_odom, '/tb3_1/')
        rospy.Subscriber('/tb3_2/odom', Odometry, self.callback_odom, '/tb3_2/')
        rospy.Subscriber('/tb3_3/odom', Odometry, self.callback_odom, '/tb3_3/')
        rospy.Subscriber('/tb3_4/odom', Odometry, self.callback_odom, '/tb3_4/')
        rospy.Subscriber('/tb3_5/odom', Odometry, self.callback_odom, '/tb3_5/')
        rospy.Subscriber('/tb3_6/odom', Odometry, self.callback_odom, '/tb3_6/')
        rospy.Subscriber('/tb3_7/odom', Odometry, self.callback_odom, '/tb3_7/')
        rospy.Subscriber('/tb3_8/odom', Odometry, self.callback_odom, '/tb3_8/')
        rospy.Subscriber('/tb3_9/odom', Odometry, self.callback_odom, '/tb3_9/')
        # rospy.Subscriber('/tb3_10/odom', Odometry, self.callback_odom,'/tb3_10/')
        # rospy.Subscriber('/tb3_11/odom', Odometry, self.callback_odom,'/tb3_11/')
        # rospy.Subscriber('/tb3_12/odom', Odometry, self.callback_odom,'/tb3_12/')
        # rospy.Subscriber('/tb3_13/odom', Odometry, self.callback_odom,'/tb3_13/')
        # rospy.Subscriber('/tb3_14/odom', Odometry, self.callback_odom,'/tb3_14/')
        # rospy.Subscriber('/tb3_15/odom', Odometry, self.callback_odom,'/tb3_15/')
        # rospy.Subscriber('/tb3_16/odom', Odometry, self.callback_odom,'/tb3_16/')
        # rospy.Subscriber('/tb3_17/odom', Odometry, self.callback_odom,'/tb3_17/')
        # rospy.Subscriber('/tb3_18/odom', Odometry, self.callback_odom,'/tb3_18/')
        # rospy.Subscriber('/tb3_19/odom', Odometry, self.callback_odom,'/tb3_19/')        
        
        self.obs = {}
        # self.obs['/tb3_0/'] = Odometry()
        self.obs['/tb3_1/'] = Odometry()
        self.obs['/tb3_2/'] = Odometry()
        self.obs['/tb3_3/'] = Odometry()
        self.obs['/tb3_4/'] = Odometry()
        self.obs['/tb3_5/'] = Odometry()
        self.obs['/tb3_6/'] = Odometry()
        self.obs['/tb3_7/'] = Odometry()
        self.obs['/tb3_8/'] = Odometry()
        self.obs['/tb3_9/'] = Odometry()
        # self.obs['/tb3_10/'] = Odometry()
        # self.obs['/tb3_11/'] = Odometry()
        # self.obs['/tb3_12/'] = Odometry()
        # self.obs['/tb3_13/'] = Odometry()
        # self.obs['/tb3_14/'] = Odometry()
        # self.obs['/tb3_15/'] = Odometry()
        # self.obs['/tb3_16/'] = Odometry()
        # self.obs['/tb3_17/'] = Odometry()
        # self.obs['/tb3_18/'] = Odometry()
        # self.obs['/tb3_19/'] = Odometry()

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
    r = rospy.Rate(4)
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
