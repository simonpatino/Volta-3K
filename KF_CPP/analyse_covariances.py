import pandas as pd
import matplotlib.pyplot as plt

# Define multiplier for standard deviation
STD_MULTIPLIER = 3  # Change this value if needed

# Load config.csv
config_df = pd.read_csv("config.csv", header=None)
output_df = pd.read_csv("output.csv", header=None)
eval_df = pd.read_csv("evaluation.csv", header=None)
time = output_df.iloc[:, 0]  # First column is time
state_dim = int(config_df.iloc[0, 0])  # Second column contains state dimension

# Load evaluation.csv

# Plot each state residual with its ±3σ bounds
plt.figure(figsize=(10, 5 * state_dim))

for i in range(state_dim):
    residual = eval_df.iloc[:, 2 * i]  # Residual for state i
    covariance = eval_df.iloc[:, 2 * i + 1]  # Covariance for state i
    std_dev = covariance**0.5  # Standard deviation
    upper_bound = STD_MULTIPLIER * std_dev
    lower_bound = -STD_MULTIPLIER * std_dev

    plt.subplot(state_dim, 1, i + 1)
    plt.plot(time, residual, label=f"Residual (State {i+1})", color='blue')
    plt.plot(time, upper_bound, linestyle='dashed', color='red', label=f"+{STD_MULTIPLIER}σ")
    plt.plot(time, lower_bound, linestyle='dashed', color='red', label=f"-{STD_MULTIPLIER}σ")
    plt.axhline(0, color='black', linewidth=0.8)
    plt.xlabel("Time")
    plt.ylabel("Residual")
    plt.title(f"Residual vs Estimated 3σ Bound (State {i+1})")
    plt.legend()
    plt.grid()

plt.tight_layout()
plt.show()