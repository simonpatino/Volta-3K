#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "matplotlibcpp.h" // Include the matplotlib-cpp header
#include "quickPlot.h" // Include the plotter header

using namespace std;
using namespace matplotlibcpp;  // Now we can use plt directly

//THIS IS HOW TO COMPILE THIS PROGRAM: g++ quickPlot.cpp -I/usr/include/python3.10 -lpython3.10 -o quickPlot && ./quickPlot

void plot_csv_data(vector<string>& filenames) {
    vector<vector<double>> time, position, velocity, acceleration;
    
    for(const string& filename : filenames) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << filename << endl;
            return;
        }
        
        string line;
        vector<double> time_data, position_data, velocity_data, acceleration_data;

        // Read data from the file
        while (getline(file, line)) {
            stringstream ss(line);
            double t, pos, vel, acc;
            char comma;

            ss >> pos >> comma >> vel >> comma >> acc >> comma >> t;

            time_data.push_back(t);
            position_data.push_back(pos);
            velocity_data.push_back(vel);
            acceleration_data.push_back(acc);
        }
        file.close();

        time.push_back(time_data);
        position.push_back(position_data);
        velocity.push_back(velocity_data);
        acceleration.push_back(acceleration_data);
    }

    // Plot Time vs Position
    figure();
    //scatter(time[0], position[0], 10.0); // 10.0 is the marker size
    if(position[0].size() == time[0].size()) {
        plot(time[0], position[0], "g-");
    } else {
        cout << "Position[0] ignored" << endl;
    }

    if(position[1].size() == time[0].size()) {
        plot(time[0], position[1], "r-");
    } else {
        cout << "Position[1] ignored" << endl;
    }

    if(position[2].size() == time[0].size()) {
        scatter(time[0], position[2], 15.0);
    } else {
        cout << "Measured position ignored" << endl;
    }
    xlabel("Time (s)");
    ylabel("Position");
    title("Time vs Position");
    grid(true);

    // Plot Time vs Velocity
    figure();
    if(velocity[0].size() == time[0].size()) {
        plot(time[0], velocity[0], "g-");
    } else {
        cout << "Velocity[0] ignored" << endl;
    }
    if(velocity[1].size() == time[0].size()) {
        plot(time[0], velocity[1], "r-");
    } else {
        cout << "Velocity[1] ignored" << endl;
    }
    xlabel("Time (s)");
    ylabel("Velocity");
    title("Time vs Velocity");
    grid(true);

    //Plot Time vs acceleration
    figure();
    if(acceleration[0].size() == time[0].size()) {
        plot(time[0], acceleration[0], "g-");
    } else {
        cout << "Acceleration[0] ignored" << endl;
    }
    if(acceleration[1].size() == time[0].size()) {
        plot(time[0], acceleration[1], "r-");
    } else {
        cout << "Acceleration[1] ignored" << endl;
    }
    xlabel("Time (s)");
    ylabel("Acceleration");
    title("Time vs Acceleration");
    grid(true);

    // Show all plots
    show();
}
