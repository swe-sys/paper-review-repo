#!/bin/bash

declare -a ALGOS=("algo1" "algo2" "algo3" "algo4")

for algo in "${ALGOS[@]}"
do
    echo "========== Running $algo =========="

    # Start roslaunch in background, redirect log
    roslaunch swarm_aggregation benchmark_run.launch algo:=$algo save_name:=$algo &
    PID=$!

    # Wait for run duration (60 sec)
    sleep 120

    # Gracefully kill the process group, giving logger time to close CSV
    echo "Stopping $algo..."
    kill -SIGINT $PID
    sleep 10
done
