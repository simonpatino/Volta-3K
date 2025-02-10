import pandas as pd
import matplotlib.pyplot as plt

def plot_state(config_file = None, real_file = None, measu_file = None, output_file = None):
    # Read configuration file
    config_data = pd.read_csv(config_file, header=None)
    state_dim = int(config_data.iloc[0, 0])  # State dimension is the first column

    # Read other files
    real_data   = pd.read_csv(real_file, header=None)
    measu_data  = pd.read_csv(measu_file, header=None)
    output_data = pd.read_csv(output_file, header=None)

    time_kf = output_data.iloc[:, 0]  # Time is the first column
    time_sim = real_data.iloc[:, 0]  # Time is the first column
    # Define variable names
    variable_names = ["Position", "Velocity", "Acceleration"]

    # Create subplots
    fig, axes = plt.subplots(state_dim, 1, figsize=(10, 6 * state_dim), sharex=True)

    # Plotting
    for i in range(state_dim):
        ax = axes[i] if state_dim > 1 else axes  # Handle single subplot case
        
        variable_name = variable_names[i] if i < len(variable_names) else f"State Variable {i+1}"
        
        # Plot real data
        ax.plot(time_sim, real_data.iloc[:, i+1], label=f"Real {variable_name}", linestyle="--")
        
        # Plot measured data
        ax.scatter(time_kf, measu_data.iloc[:, i], label=f"Measured {variable_name}", alpha=0.2)
        
        # Plot Kalman filter output
        ax.plot(time_kf, output_data.iloc[:, i+1], label=f"Kalman Filter Output {variable_name}")
        
        # Labels and legend
        ax.set_ylabel(variable_name)
        ax.set_title(f"{variable_name} vs Time")
        ax.legend()
        ax.grid(True)

    # Common x-axis label
    plt.xlabel("Time")

    # Adjust layout
    plt.tight_layout()

    # Save the plot
    plt.savefig("all_states_plot.png")

    # Show the plot
    plt.show()




if __name__ == '__main__':
    # File names
    conf_file = "config.csv"
    r_file = "real.csv"
    meas_file = "measurements.csv"
    opt_file = "output.csv"
    plot_state(config_file = conf_file, real_file = r_file, measu_file = meas_file, output_file = opt_file)