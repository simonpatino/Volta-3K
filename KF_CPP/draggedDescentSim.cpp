#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>

using namespace std;

// Function to compute acceleration (drag + gravity)
double computeAcceleration(double velocity, double Cd, double rho, double A, double mass, double gravity) {
    double dragForce = 0.5 * Cd * rho * A * velocity * velocity * (velocity > 0 ? 1 : -1);
    return gravity - (dragForce / mass);
}

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
    double time = 3;
    const double timeStep = 0.05; // seconds
    const double totalTime = 150.0; // seconds
    double position = 703.2;
    double velocity = 291.0; // Initial velocity
    const double gravity = -9.8; // m/s²
    double Cd = 0.47;  // Drag coefficient
    const double rho = 1.112; // Air density (kg/m³)
    const double A = 0.023787;  // Cross-sectional area (m²)
    const double mass = 30.747; // Mass (kg)
    const double noiseVarPos = 0.0; // Noise variance
    const double noiseVarVel = 0.0; // Noise variance
    const double noiseVarAcc = 0.0; // Noise variance
    bool apogee = true;

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
        if (velocity < 0) {
            if (apogee) {
                cout << "Apogee is at: " << position << endl;
                apogee = false;
            }
            Cd = 1.16 + 0.47;  
            if (position < 1200) {
                Cd = 1.16 + 0.47+2.92;  
            }
        }
        if (position < 0) {
            time = totalTime;
        }

        // RK4 Integration
        double k1_vel = computeAcceleration(velocity, Cd, rho, A, mass, gravity);
        double k1_pos = velocity;

        double k2_vel = computeAcceleration(velocity + 0.5 * timeStep * k1_vel, Cd, rho, A, mass, gravity);
        double k2_pos = velocity + 0.5 * timeStep * k1_vel;

        double k3_vel = computeAcceleration(velocity + 0.5 * timeStep * k2_vel, Cd, rho, A, mass, gravity);
        double k3_pos = velocity + 0.5 * timeStep * k2_vel;

        double k4_vel = computeAcceleration(velocity + timeStep * k3_vel, Cd, rho, A, mass, gravity);
        double k4_pos = velocity + timeStep * k3_vel;

        // Update velocity and position using RK4
        velocity += (timeStep / 6.0) * (k1_vel + 2 * k2_vel + 2 * k3_vel + k4_vel) + noise_vel(gen);
        position += (timeStep / 6.0) * (k1_pos + 2 * k2_pos + 2 * k3_pos + k4_pos) + noise_pos(gen);

        // Compute noisy acceleration
        double noisyAcceleration = computeAcceleration(velocity, Cd, rho, A, mass, gravity) + noise_acc(gen);

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