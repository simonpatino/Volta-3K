#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include "quickPlot.h" // Include the header to call plot_csv_data

using namespace std;

// Function to write data to a CSV file
void writeToCSV(const string &filename, const vector<double> &time, const vector<double> &position, 
                const vector<double> &velocity) {
    ofstream file(filename);

    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return;
    }

/*     // Write the header
    file << "Time,Position,Velocity,Acceleration\n"; */

    // Write data
    for (size_t i = 0; i < time.size(); ++i) {
        file << fixed << setprecision(3) 
             << time[i] << "," 
             << position[i] << "," 
             << velocity[i] << "\n";
    }

    file.close();
}

int main() {
    // Simulation parameters
    double time = 0.0;
    const double timeStep = .25; // seconds
    const double totalTime = 40.0; // seconds
    double position = 0.0;
    const double constantVelocity = -.8; // m/s
    const double noiseVar = 1.5; // Standard deviation of Gaussian noise

    // Random number generator setup for Gaussian noise
    random_device rd; //An actual random shit given by the hardware entropy
    mt19937 gen(rd()); //Mersenne Twister 19937 algorithm. Se genera una instncia del algoritmo con una seed "completamente" aleatoria
    normal_distribution<> noise(0.0, sqrt(noiseVar)); //Se define el default (double) de la template class y se crea la instancia noise.

    // Initialize state variables

    // Vectors to store results
    vector<double> timeValues; //Inicializa los objeto vectores en su forma double
    vector<double> positionValues;
    vector<double> velocityValues;

    // Simulation loop
    while (time <= totalTime) {
        // Calculate noisy acceleration
        double noisyVelocity = constantVelocity + noise(gen);

        // Store values
        timeValues.push_back(time);
        positionValues.push_back(position);
        velocityValues.push_back(noisyVelocity);

        // Update state using simple integration
        position += noisyVelocity * timeStep;


        // Increment time
        time += timeStep;
    }

    vector<string> filenames = {"falling_object.csv"};
    // Write results to a CSV file
    writeToCSV(filenames[0], positionValues, velocityValues, timeValues);

    cout << "Simulation complete. Data saved to " << filenames[0] << endl;
    plot_csv_data(filenames); // Call the function
    return 0;
}
