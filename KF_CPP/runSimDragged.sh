#!/bin/bash

echo "Running dragged descent simulation"

# Run the make command
if make -f MakeSimDragged; then
    echo "Compilation successful."

    # Run the simulation program
    if ./draggedDescentSim; then
        echo "Simulation completed successfully."

        # Run the Python script for plotting
        if python3 plot_sim.py; then
            echo "Plot generated successfully."
        else
            echo "Error: Failed to run plot_sim.py"
            exit 3
        fi
    else
        echo "Error: draggedDescentSim failed to run."
        exit 2
    fi
else
    echo "Error: make command failed."
    exit 1
fi