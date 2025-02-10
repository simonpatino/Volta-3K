#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>

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
    const double timeStep = .05; // seconds
    const double totalTime = 200.; // seconds
    double position = 0.0;
    double velocity = 0.0; // Initial velocity
    const double gravity = -9.8; // m/s²
    const double Cd = 0.1;  // Drag coefficient
    const double rho = 1.225; // Air density (kg/m³)
    const double A = 0.01;  // Cross-sectional area (m²)
    const double mass = 5.0; // Mass (kg)
    const double noiseVarPos = 250.0; // Noise variance
    const double noiseVarVel = 5; // Noise variance
    const double noiseVarAcc = 10; // Noise variance

    // Random number generator setup for Gaussian noise
    mt19937 gen(42);
    normal_distribution<> noise_pos(0.0, sqrt(noiseVarPos));
    normal_distribution<> noise_vel(0.0, sqrt(noiseVarVel));
    normal_distribution<> noise_acc(0.0, sqrt(noiseVarAcc));

    // Vectors to store results
    vector<double> timeValues;
    vector<double> positionValues;
    vector<double> velocityValues;
    vector<double> accelerationValues;

    // Simulation loop
    while (time <= totalTime) {
        // Compute drag force
        double dragForce = 0.5 * Cd * rho * A * velocity * velocity * (velocity > 0 ? 1 : -1);
        double dragAcceleration = dragForce / mass;

        // Compute total acceleration with noise
        double noisyAcceleration = gravity - dragAcceleration + noise_acc(gen);

        // Update velocity and position
        velocity += noisyAcceleration * timeStep + noise_vel(gen);
        position += velocity * timeStep + noise_pos(gen);

        // Store values
        timeValues.push_back(time);
        positionValues.push_back(position);
        velocityValues.push_back(velocity);
        accelerationValues.push_back(noisyAcceleration);

        time += timeStep;
    }

    // Write config.csv
    ofstream configFile("config.csv");
    if (!configFile.is_open()) {
        cerr << "Failed to open config.csv" << endl;
        return 1;
    }
    for (size_t i = 0; i < timeValues.size(); ++i) {
        configFile << fixed << setprecision(3) << "3,2\n"; // x_dim=3, z_dim=2
    }
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