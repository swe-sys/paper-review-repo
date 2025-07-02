#!/usr/bin/env python3

import rospy
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

class InterRobotLogger:
    def __init__(self, robot_names, filename):
        log_dir = os.path.expanduser("~/swarm_logs1")
        os.makedirs(log_dir, exist_ok=True)
        self.save_path = os.path.join(log_dir, filename)
        print(f"[Logger] Writing CSV to: {self.save_path}")

        self.positions = {name: np.array([0.0, 0.0]) for name in robot_names}
        self.dist_log = []
        self.start_time = rospy.Time.now().to_sec()

        self.csv_file = open(self.save_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'avg_dist', 'min_dist', 'max_dist'])

        for name in robot_names:
            rospy.Subscriber(f"/{name}/odom", Odometry, self.odom_cb, name)

    def odom_cb(self, msg, name):
        self.positions[name] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def compute_metrics(self):
        pos_list = list(self.positions.values())
        dists = [np.linalg.norm(pos_list[i] - pos_list[j]) for i in range(len(pos_list)) for j in range(i+1, len(pos_list))]
        return np.mean(dists), np.min(dists), np.max(dists)

    def run(self):
        rate = rospy.Rate(5)
        try:
            while not rospy.is_shutdown():
                t = rospy.Time.now().to_sec() - self.start_time
                avg, min_d, max_d = self.compute_metrics()
                self.writer.writerow([t, avg, min_d, max_d])
                self.dist_log.append((t, avg))
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                rate.sleep()
        finally:
            self.csv_file.close()
            print(f"[Logger] Final CSV written to: {self.save_path}")
            self.plot_results()

    def plot_results(self):
        if not self.dist_log:
            print("No data to plot.")
            return
        t, d = zip(*self.dist_log)
        plt.figure()
        plt.plot(t, d, label=os.path.basename(self.save_path))
        plt.xlabel("Time (s)")
        plt.ylabel("Avg Distance (m)")
        plt.title("Inter-Robot Distance")
        plt.grid(True)
        plt.legend()
        plt.savefig(self.save_path.replace(".csv", ".png"))
        plt.show()

if __name__ == '__main__':
    rospy.init_node("inter_robot_logger")

    # ✅ READ PARAM AFTER init_node()
    filename = rospy.get_param("~save_path", "log.csv")
    robots = [f"tb3_{i}" for i in range(7)]
    logger = InterRobotLogger(robots, filename)
    try:
        logger.run()
    except rospy.ROSInterruptException:
        logger.csv_file.close()
