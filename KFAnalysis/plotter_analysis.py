import pandas as pd
import matplotlib.pyplot as plt

def plot_state(config_file=None, real_file=None, measu_file=None, output_file=None):
    # Read configuration file
    config_data = pd.read_csv(config_file, header=None)
    state_dim = int(config_data.iloc[0, 0])  # State dimension is now in the first column

    # Read real data file
    real_data = pd.read_csv(real_file, header=None)
    real_time = real_data.iloc[:, 0]  # Time is the first column in real data
    real_states = real_data.iloc[:, 1:]  # Position, Velocity, Acceleration are the remaining columns

    # Read measurement data file
    measu_data = pd.read_csv(measu_file, header=None)
    # Measurement data only contains Position, Velocity, Acceleration (no time column)

    # Read output data file
    output_data = pd.read_csv(output_file, header=None)
    output_time = output_data.iloc[:, 0]  # Time is the first column in output data
    output_states = output_data.iloc[:, 1:]  # Position, Velocity, Acceleration are the remaining columns

    # Check dimensions
    if not (real_states.shape[1] == measu_data.shape[1] == output_states.shape[1] == state_dim):
        raise ValueError("Mismatch between state dimension and data file columns.")

    # Define variable names
    variable_names = ["Position", "Velocity", "Acceleration"]

    # Create subplots
    fig, axes = plt.subplots(state_dim, 1, figsize=(10, 6 * state_dim), sharex=True)

    # Plotting
    for i in range(state_dim):
        ax = axes[i] if state_dim > 1 else axes  # Handle single subplot case
        
        variable_name = variable_names[i] if i < len(variable_names) else f"State Variable {i+1}"
        
        # Plot real data (using real_time)
        ax.plot(real_time, real_states.iloc[:, i], label=f"Real {variable_name}", linestyle="--")
        
        # Plot measured data (using output_time)
        ax.scatter(output_time, measu_data.iloc[:, i], label=f"Measured {variable_name}", alpha=0.7)
        
        # Plot Kalman filter output (using output_time)
        ax.plot(output_time, output_states.iloc[:, i], label=f"Kalman Filter Output {variable_name}")
        
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
    conf_file = "kf_config.csv"
    r_file = "kf_real.csv"
    meas_file = "kf_measu.csv"
    opt_file = "kf_output.csv"
    plot_state(config_file=conf_file, real_file=r_file, measu_file=meas_file, output_file=opt_file)