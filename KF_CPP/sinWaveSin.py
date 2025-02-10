#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>

using namespace std;

// Function to write data to a CSV file
void writeToCSV(const string &filename, const vector<double> &col1, const vector<double> &col2, 
                const vector<double> &col3, const vector<double> &col4) {
    ofstream file(filename);

    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return;
    }

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
    const double timeStep = 0.01; // seconds
    const double totalTime = 200.0; // seconds
    double position = 0.0;
    double velocity = 0.0; // Initial velocity
    const double amplitude = 10.0; // Amplitude of sine wave velocity
    const double frequency = 0.1; // Frequency of sine wave (Hz)
    const double noiseVar = 10.0; // Noise variance

    // Random number generator setup for Gaussian noise
    mt19937 gen(42);
    normal_distribution<> noise(0.0, sqrt(noiseVar));

    // Vectors to store results
    vector<double> timeValues;
    vector<double> positionValues;
    vector<double> velocityValues;
    vector<double> accelerationValues;

    // Simulation loop
    while (time <= totalTime) {
        // Compute sinusoidal velocity
        velocity = amplitude * sin(2 * M_PI * frequency * time);
        
        // Compute acceleration as the derivative of velocity (cosine function)
        double acceleration = 2 * M_PI * frequency * amplitude * cos(2 * M_PI * frequency * time) + noise(gen);
        
        // Update position
        position += velocity * timeStep;

        // Store values
        timeValues.push_back(time);
        positionValues.push_back(position);
        velocityValues.push_back(velocity);
        accelerationValues.push_back(acceleration);

        time += timeStep;
    }

    // Write config.csv
    ofstream configFile("config.csv");
    if (!configFile.is_open()) {
        cerr << "Failed to open config.csv" << endl;
        return 1;
    }
    configFile << "3,2\n"; // x_dim=3, z_dim=2
    configFile.close();

    // Write real.csv
    ofstream realFile("real.csv");
    if (!realFile.is_open()) {
        cerr << "Failed to open real.csv" << endl;
        return 1;
    }
    for (size_t i = 0; i < positionValues.size(); ++i) {
        realFile << fixed << setprecision(3) 
                 << timeValues[i] << ","
                 << positionValues[i] << "," 
                 << velocityValues[i] << "," 
                 << accelerationValues[i] << "\n";
    }
    realFile.close();

    cout << "Simulation complete. Data saved to config.csv and real.csv" << endl;
    return 0;
}
