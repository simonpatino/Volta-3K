#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include <map>
#include "LinearKF.h"
#include <Eigen/Dense>
#include "quickPlot.h"

using namespace std; 
using namespace Eigen; 

string filename = "./falling_object.csv";

double current_time = 0;
double dt;
int iteration = 0;

bool getRowData(int row, double data[4]) {
    ifstream file(filename);
    if (!file.is_open()) {
        return false;  // Failed to open file
    }

    string line;
    int current_row = 0;

    // Read through the file line by line
    while (getline(file, line)) {
        // When we reach the desired row
        if (current_row == row) {
            stringstream ss(line);
            string value;
            int col_index = 0;

            // Read the values from the CSV row and fill the data array
            while (getline(ss, value, ',')) {
                if (col_index < 4) {
                    data[col_index] = stod(value);  // Convert string to double
                }
                col_index++;
            }
            file.close();
            return true;  // Successfully read the row
        }
        current_row++;
    }
    file.close();
    return false;  // Row index out of range
}

double readPositionSensor(int i, double var) {
    double data[4];
    getRowData(i, data);

    // Set up a random number generator with a constant seed for reproducibility
    static mt19937 gen(42); // Constant seed
    static normal_distribution<double> dist(0.0, sqrt(var));

    // Add Gaussian noise to data[1] (position)
    double noisyValue = data[0] + dist(gen);

    return noisyValue;
}

/*
    Assumes a constant dt. Uses the data from the csv to calculate the time step of the simulation
 */
double getdt() {
    double arr1[4], arr2[4];
    getRowData(1, arr1);
    getRowData(2, arr2);
    return (arr2[2] - arr1[2]);
}

void save_vector_to_csv(const VectorXd& x, const string& filename) {
    // Static map to track if a file has been written to for the first time
    static map<string, bool> first_call_map;

    ofstream file;

    // Check if this is the first call for the given filename
    if (first_call_map[filename] == false) {
        file.open(filename, ios::trunc);  // Open in truncate mode to erase contents
        first_call_map[filename] = true;  // Mark as initialized
    } else {
        file.open(filename, ios::app);  // Open in append mode
    }

    if (file.is_open()) {
        for (int i = 0; i < x.size(); ++i) {
            file << x[i];  // Write the value of each element
            if (i < x.size() - 1) {
                file << ",";  // Separate elements by a comma
            }
        }
        file << endl;  // Add a new line after the row
        file.close();  // Close the file
    } else {
        cerr << "Error: Could not open the file!" << endl;
    }
}

int main() {
    int x_dim = 2;
    int z_dim = 1;
    int u_dim = 1;
    double process_variance = 0.3872;
    double set_pos_measurement_variance = 10;
    double actual_pos_measurement_variance = 10;

    dt = getdt();

    cout << "Delta_t is " << dt << endl;
    
    LinearKF kf(x_dim, z_dim, u_dim);  // Creating the object with default dimensions

    VectorXd initial_state(x_dim);
    MatrixXd initial_covariance(x_dim,x_dim);
    initial_state << 0, -5.0;
    initial_covariance << 500, 0,
                             0, 500;

    MatrixXd F(x_dim,x_dim), Q(x_dim,x_dim), H(z_dim, x_dim), R(z_dim,z_dim);
    F << 1, dt,
         0, 1;

    H << 1, 0;

    R << set_pos_measurement_variance;

/*     Q << 0.2, 0,
           0, 0.2; */

    kf.initialize_state(initial_state, initial_covariance);
    //kf.set_Q(Q);
    kf.set_Q_discrete_white_noise(x_dim, dt, process_variance);
    kf.set_F(F);
    kf.set_H(H);
    kf.set_R(R);

    VectorXd z(z_dim);
    while(iteration < 160) {
        save_vector_to_csv(kf.get_x(), "output.csv");
        z(0) = readPositionSensor(iteration, actual_pos_measurement_variance);
        kf.predict();
        kf.update(z);
        save_vector_to_csv(z, "measurements.csv");
        iteration++;
    }

    save_vector_to_csv(kf.get_x(), "output.csv");
    save_vector_to_csv(z, "measurements.csv");


    //Plotting the data:
    vector<string> filenames = {"falling_object.csv", "output.csv", "measurements.csv"};
    plot_csv_data(filenames); // Call the function

    return 0;
}