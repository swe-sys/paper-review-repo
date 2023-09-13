#!/usr/bin/python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from rosgraph_msgs.msg import Clock

robots = []
ROBOT_COUNT = 0
N = 6

clusters = []
CLUSTER_COUNT = 0
CLUSTERS_DELETED = 0

COMM_RANGE = 4
ROBOT_RADIUS = .1

VELOCITY = 0.18 # max vel
OMEGA = 0.5 # max angular vel

dt = 1

WORLD_LENGTH = 6

probs = np.zeros((N,2)) #stores p_join and p_leave for every cluster of size k   # k<=N
probs = np.array([[1,0]]*(N+1))


class Robot:

    def __init__(self, id=None, pos=None) -> None:
        global ROBOT_COUNT
        if id==None:
            id = ROBOT_COUNT
        self.id = id
        if pos==None:
            pos = ROBOT_COUNT*1000*np.random.rand(3)
        self.pos = np.array(pos) # x, y, alpha
        print(self, self.pos)
        ROBOT_COUNT = ROBOT_COUNT + 1
        robots.append(self)
        self.cluster = None

        rospy.Subscriber(f'/tb3_{ROBOT_COUNT}/odom',
                         Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher(
            f'/tb3_{ROBOT_COUNT}/cmd_vel', Twist, queue_size=10)
        print(self," init done")

    def __repr__(self):
        return "robot %s"%(self.id+1)


    def dynamics(self):

        num = np.random.rand()

        if self.cluster == None:

            new_omega = OMEGA*np.random.rand()

            new_alpha = self.pos[2]+new_omega*dt
            #return self.pos + dt*np.array([VELOCITY*np.cos(new_alpha), VELOCITY*np.sin(new_alpha), new_omega])
            return np.array([VELOCITY*np.cos(new_alpha), VELOCITY*np.sin(new_alpha), new_omega])
            
        # leave
        
        location_estimator = np.array([0,0], dtype=float)
        for i in range(self.cluster.num_robots):
            vec = self.pos[:2]-self.cluster.robots[i].pos[:2]
            if (np.linalg.norm(vec)<COMM_RANGE):
                location_estimator += vec

        if np.linalg.norm(location_estimator)>ROBOT_RADIUS:
            #robot near boundary
            
            if num < probs[self.cluster.num_robots,1]:
                vec = self.pos[:2]-self.cluster.pos[:2]
                new_alpha = np.arctan2(vec[1], vec[0])
                new_omega = (new_alpha-self.pos[2])/dt
                #return self.pos + dt*np.array([VELOCITY*np.cos(new_alpha), VELOCITY*np.sin(new_alpha), new_omega])
                return np.array([VELOCITY*np.cos(new_alpha), VELOCITY*np.sin(new_alpha), new_omega])


        # stay
        #return self.pos
        return np.zeros(3)
    
    def calc_and_pub_vel(self):
        vel = self.dynamics()
        cmd_vel = Twist()
        cmd_vel.linear.x = np.linalg.norm(vel[:2])
        print(cmd_vel.linear.x, cmd_vel.angular.z)
        self.vel_pub.publish(cmd_vel) 

    def odom_callback(self, data):
        pos = data.pose.pose
        position = pos.position

        # global x and y positions
        x, y = position.x, position.y

        # alpha relative to axis parallel to global x
        quaternion = pos.orientation
        _, __, alpha = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])  # alpha \in [-pi,pi]
        if alpha < 0:
            alpha = 2*np.pi + alpha  # alpha \in [0,2pi]

        self.pos = np.array([x,y,alpha])

class Cluster:

    def __init__(self, first_robot) -> None:
        global CLUSTER_COUNT
        self.num_robots = 0
        self.robots = []
        self.id = CLUSTER_COUNT
        CLUSTER_COUNT = CLUSTER_COUNT + 1
        print(self," created")
        self.addRobot(first_robot)
        

    def __repr__(self):
        return "cluster %s" % (self.id+1)

    def addRobot(self, robot):
        self.num_robots = self.num_robots + 1
        self.robots.append(robot)
        self.robots[self.num_robots-1].cluster = self
        self.calcState()
        print("Added ", robot, " to ", self)

    def delRobot(self, robot):

        if self.num_robots==0:
            return False

        self.num_robots = self.num_robots - 1

        i = 0
        while self.robots[i].id != robot.id and i<self.num_robots:
            i += 1

        del(self.robots[i])

        i = 0
        while robots[i].id != robot.id and i<ROBOT_COUNT:
            i += 1

        robots[i].cluster = None

        print("Deleted ", robot, " from ", self)

        if self.num_robots==0:
            i = 0
            while clusters[i].id != self.id and i<len(clusters):
                i += 1
            del(clusters[i])
            global CLUSTERS_DELETED
            CLUSTERS_DELETED = CLUSTERS_DELETED + 1
            print(self, " deleted")
        else:
            self.calcState()

    def mergeCluster(self, cluster):
        for i in cluster.robots:
            self.addRobot(i)
            cluster.delRobot(i)

        print(cluster, " merged into ", self)
    
    def calcState(self):
        x_mean, y_mean = 0, 0
        for i in self.robots:
            x_mean = x_mean + i.pos[0]
            y_mean = y_mean + i.pos[1]
        self.pos = np.array([x_mean, y_mean])/self.num_robots

        
def calculate_clusters():
    print("running calculations")
    for i in range(ROBOT_COUNT):
        for j in range(ROBOT_COUNT):
            if (i<j):
                d_ij = np.linalg.norm(robots[i].pos[:2]-robots[j].pos[:2])
                print(robots[i], "      ", robots[j],"      ", d_ij, "      ", end="")
                if (d_ij<COMM_RANGE):
                    print("found " , robots[i]," and ",robots[j] , " close to each other")
                    if (robots[i].cluster != None) and (robots[j].cluster != None):
                        print(
                            "found " , robots[i]," and ",robots[j] , " in clusters")

                        if (robots[i].cluster.id != robots[j].cluster.id):
                            print(
                                "found " , robots[i]," and ",robots[j] , " in different clusters")

                            robots[i].cluster.mergeCluster(robots[j].cluster)
                        else:
                            print("robots found in same cluster")

                    else:
                        #join
                        
                        join_prob = np.random.rand()
                        print(join_prob)

                        if (robots[i].cluster == None) and (robots[j].cluster == None):
                            print(
                                "found " , robots[i]," and ",robots[j] , " in no cluster")

                            if join_prob < probs[0,0]:
                                clusters.append(Cluster(robots[i]))
                                clusters[CLUSTER_COUNT-1-CLUSTERS_DELETED].addRobot(robots[j])

                        elif (robots[i].cluster != None) and (robots[j].cluster == None):
                            print(
                                "found " , robots[i]," and ",robots[j] , " in one cluster and other in none")

                            if join_prob < probs[robots[i].cluster.num_robots,0]:
                                robots[i].cluster.addRobot(robots[j])

                        else:
                            print(
                                "found " , robots[i]," and ",robots[j] , " in no cluster and other in one")

                            if join_prob < probs[robots[j].cluster.num_robots,0]:
                                robots[j].cluster.addRobot(robots[i])
                robots[j].calc_and_pub_vel()
        print("")
        print(robots[i], "        ", end="")
        robots[i].calc_and_pub_vel()
        print()
    for i in range(CLUSTER_COUNT-CLUSTERS_DELETED):
        print(clusters[i], end=": ")
        for j in range(clusters[i].num_robots):
            print(clusters[i].robots[j], end=" ")
        print("")

                                
def run():
    tsub = rospy.Subscriber('/clock', Clock, tCallback)
    rospy.spin()
    

def tCallback(_):
    calculate_clusters()


if __name__ == '__main__':
    ROBOT_COUNT = 0
    rospy.init_node('my_node', anonymous=True)
    Robot()
    Robot()
    Robot()
    Robot()
    Robot()
    Robot()

    try:
        run()
    except rospy.ROSInterruptException:
        pass

