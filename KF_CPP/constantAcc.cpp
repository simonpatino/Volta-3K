#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include "quickPlot.h" // Include the header to call plot_csv_data

using namespace std;

// Function to write data to a CSV file
void writeToCSV(const string &filename, const vector<double> &col1, const vector<double> &col2, 
                const vector<double> &col3, const vector<double> &col4) {
    ofstream file(filename);

    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return;
    }

    // Write data
    for (size_t i = 0; i < col1.size(); ++i) {
        file << fixed << setprecision(3) 
             << col1[i] << "," 
             << col2[i] << "," 
             << col3[i] << ","
             << col4[i] << "\n";
    }

    file.close();
}

int main() {
    // Simulation parameters
    double time = 0.0;
    const double timeStep = .8; // seconds
    const double totalTime = 100; // seconds
    double position = 0.0;
    double velocity = 0.0; // Initial velocity (could be zero)
    const double constantAcceleration = -1.5; // m/s² (gravity for example)
    const double noiseVar = 50; // Standard deviation of Gaussian noise

    // Constants for config.csv
    const double x_dim = 3.0; // Example constant value for x_dim
    const double z_dim = 1.0;  // Example constant value for z_dim

    // Random number generator setup for Gaussian noise
    random_device rd; // Hardware entropy for random seed
    mt19937 gen(42); // Mersenne Twister 19937 algorithm for random number generation
    normal_distribution<> noise(0.0, sqrt(noiseVar)); // Gaussian noise distribution

    // Vectors to store results
    vector<double> timeValues;
    vector<double> positionValues;
    vector<double> velocityValues;
    vector<double> accelerationValues; // To store acceleration

    // Simulation loop
    while (time <= totalTime) {
        // Apply noise to the acceleration
        double noisyAcceleration = constantAcceleration + noise(gen);

        // Update velocity with noisy acceleration (velocity += acceleration * timeStep)
        velocity += noisyAcceleration * timeStep;

        // Store values
        timeValues.push_back(time);
        positionValues.push_back(position);
        velocityValues.push_back(velocity);
        accelerationValues.push_back(noisyAcceleration); // Store noisy acceleration

        // Update position with velocity (position += velocity * timeStep)
        position += velocity * timeStep;

        // Increment time
        time += timeStep;
    }

    // Write config.csv (time, x_dim, z_dim)
    ofstream configFile("config.csv");
    if (!configFile.is_open()) {
        cerr << "Failed to open config.csv" << endl;
        return 1;
    }
    for (size_t i = 0; i < timeValues.size(); ++i) {
        configFile << fixed << setprecision(3) 
                   << timeValues[i] << "," 
                   << x_dim << "," 
                   << z_dim << "\n";
    }
    configFile.close();

    // Write real.csv (position, velocity, acceleration)
    ofstream realFile("real.csv");
    if (!realFile.is_open()) {
        cerr << "Failed to open real.csv" << endl;
        return 1;
    }
    for (size_t i = 0; i < positionValues.size(); ++i) {
        realFile << fixed << setprecision(3) 
                 << positionValues[i] << "," 
                 << velocityValues[i] << "," 
                 << accelerationValues[i] << "\n";
    }
    realFile.close();

    cout << "Simulation complete. Data saved to config.csv and real.csv" << endl;

    // Plot the data (assuming plot_csv_data can handle multiple files)
    vector<string> filenames = {"config.csv", "real.csv"};

    return 0;
}