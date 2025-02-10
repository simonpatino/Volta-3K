#!/bin/bash

# Run the make command
make -f MakefileSimAcc

# Check if the previous command was successful
if [ $? -eq 0 ]; then
    # Run the constantAcc program
    ./constantAcc
    
    # Check if constantAcc was successful
    if [ $? -eq 0 ]; then
        # Run the Python script
        python3 plot_sim.py
    else
        echo "Error: constantAcc failed to run."
    fi
else
    echo "Error: make command failed."
fi
