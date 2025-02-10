#!/bin/bash

# Compile the program using the Makefile
if make -f MakefileUKF; then
    echo "Compilation successful."

    # Run the compiled program
    if ./driverUKF; then
        echo "Simulation ran successfully."

        # Run the Python analysis script
        if python3 plotter_analysis.py; then
            echo "Python analysis script executed successfully. Analysing..."
	    python3 analyse_covariances.py
        else
            echo "Error: Python analysis script failed to execute."
        fi
    else
        echo "Error: Simulation failed to run."
    fi
else
    echo "Error: Compilation failed."
fi
