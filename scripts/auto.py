#!/usr/bin/env python3
import os
import subprocess

# Number of times to run the roslaunch file
num_runs = 5

# Path to the roslaunch file
roslaunch_file = "/home/sweksha/ros_ws/src/swarm_aggregation/launch/hexagon.launch"

# Create a parent directory to store the results
parent_directory = "Six_Robots"
os.makedirs(parent_directory, exist_ok=True)

for i in range(num_runs):
    # Create a directory for each run
    run_directory = os.path.join(parent_directory, f"run_{i}")
    os.makedirs(run_directory, exist_ok=True)

    # Run roslaunch and capture the output
    with open(os.path.join(run_directory, "output.log"), "w") as log_file:
        subprocess.run(["roslaunch", roslaunch_file], stdout=log_file, stderr=subprocess.STDOUT, text=True)

    print(f"Run {i} completed. Output saved in {run_directory}")

    if
