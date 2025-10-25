#!/usr/bin/env python3

# import rospy
# import os
# import rospkg

# for i in range(2):
#     dirname = rospkg.RosPack().get_path('swarm_aggregation')
#     os.system(f"rm -rf {dirname}/Data/Data{i}/*")
#     os.system(f"rosparam set /iteration/ {i}")
#     os.system("roslaunch swarm_aggregation hexagon.launch")

import signal
import subprocess
import time
import rospkg
import os

trials = 30
dirname = rospkg.RosPack().get_path('swarm_aggregation')
time_csv = f'{dirname}/scripts/time.csv'

MAX_TRIAL_TIME = 600  # 6 minutes
GRACEFUL_TIMEOUT = 10  # seconds to wait before force kill

for i in range(trials):
    os.system(f"rm -rf {dirname}/Data/Data{i}/*")
    os.system(f"rosparam set /iteration {i}")

    print(f"\n[INFO] Starting trial {i}...")
    proc = subprocess.Popen(["roslaunch", "swarm_aggregation", "hexagon.launch"])

    start_time = time.time()
    last_size = os.path.getsize(time_csv) if os.path.exists(time_csv) else 0
    converged = False

    while not converged:
        time.sleep(3)
        elapsed = time.time() - start_time

        if os.path.exists(time_csv):
            new_size = os.path.getsize(time_csv)
            if new_size > last_size:
                converged = True
                print(f"[INFO] Trial {i} converged based on time.csv update.")
                break

        if elapsed > MAX_TRIAL_TIME:
            print(f"[WARN] Trial {i} exceeded {MAX_TRIAL_TIME/60:.1f} minutes — forcing termination.")
            break

    # Try graceful termination
    proc.terminate()
    try:
        proc.wait(timeout=GRACEFUL_TIMEOUT)
    except subprocess.TimeoutExpired:
        print(f"[ERROR] Trial {i} did not terminate gracefully. Killing all ROS/Gazebo processes...")
        # Force kill all ROS-related processes
        os.system("pkill -9 -f roslaunch")
        os.system("pkill -9 -f gazebo")
        os.system("pkill -9 -f gzserver")
        os.system("pkill -9 -f gzclient")
        # os.system("pkill -9 -f rosmaster")
        # os.system("pkill -9 -f rosout")

    print(f"[INFO] Trial {i} complete.\n")
    time.sleep(5)

print("[INFO] All trials complete.")