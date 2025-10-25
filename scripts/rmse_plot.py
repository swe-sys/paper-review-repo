#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Configuration
log_dir = os.path.expanduser("~/swarm_logs")
# File name keys (used to load CSVs)
file_keys = ['algo1', 'algo2', 'algo3', 'algo4']
# Display names for x-axis
algos = ['PSO', 'Boids', 'BeeClust', 'Proposed']
colors = ['r', 'g', 'b', 'k']
ideal_avg_distance = 2.0  # Adjust this target as needed

def compute_rmse(series, ideal):
    errors = series - ideal
    mse = np.mean(errors**2)
    return np.sqrt(mse)

# Collect RMSE values
rmse_values = []

for key, label in zip(file_keys, algos):
    csv_path = os.path.join(log_dir, f"{key}_distances.csv")
    if not os.path.exists(csv_path):
        print(f"[Warning] Missing: {csv_path}")
        rmse_values.append(np.nan)
        continue

    df = pd.read_csv(csv_path)
    if 'min_dist' not in df.columns:
        print(f"[Error] 'min_dist' column not found in {csv_path}")
        rmse_values.append(np.nan)
        continue

    rmse = compute_rmse(df['min_dist'], ideal_avg_distance)
    print(f"[INFO] RMSE for {label}: {rmse:.3f}")
    rmse_values.append(rmse)

# Plot
plt.figure(figsize=(8, 6))
bars = plt.bar(algos, rmse_values, color=colors)
plt.title("RMSE of Average Inter-Robot Distance per Algorithm")
plt.ylabel("RMSE (m)")
plt.xlabel("Algorithm")
plt.grid(axis='y')

# Annotate bars
for bar, rmse in zip(bars, rmse_values):
    if not np.isnan(rmse):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                 f"{rmse:.3f}", ha='center', va='bottom', fontsize=10)

plt.tight_layout()
plt.savefig(os.path.join(log_dir, "rmse_comparison_plot.png"))
plt.show()