#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include <map>
#include "UKF.h"
#include <Eigen/Dense>
#include "quickPlot.h"
#include "Sigma.h"

using namespace std; 
using namespace Eigen; 

string real_filename = "./real.csv";

double current_time = 0;
double prev_time = 0;
double dt;
int iteration = 0;
const int x_dim = 3;
const int z_dim = 2;
const int u_dim = 1;
double dt_sum = 0;
double kf_delay = 1.5;

//Physics data
const double gravity = -9.8; // m/s²
const double Cd = 0.1;  // Drag coefficient
const double rho = 1.225; // Air density (kg/m³)
const double A = 0.01;  // Cross-sectional area (m²)
const double mass = 5.0; // Mass (kg)


vector<string> file_cache;  // Cache to store all rows of the CSV file

// Function to load the entire file into memory (only called once)
void loadFileIntoMemory(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open " << filename << endl;
        return;
    }

    string line;
    while (getline(file, line)) {
        file_cache.push_back(line);  // Store each row in the vector
    }
    file.close();

    cout << "File loaded into memory. Total rows: " << file_cache.size() << endl;
}


// Optimized getRowData using in-memory storage
bool getRowData(int row, double data[4]) {
    if (row < 0 || row >= file_cache.size()) {
        cerr << "Error: Row index out of range" << endl;
        return false;
    }

    stringstream ss(file_cache[row]);
    string value;
    int col_index = 0;

    while (getline(ss, value, ',')) {
        if (col_index < 4) { // Read as many columns as needed
            data[col_index] = stod(value);
        }
        col_index++;
    }

    return true;
}

double* readSensor(int i, double varBaro, double varAccelerometer, VectorXd* ground) {
    double* measurements = new double[2];
    double real_data[4];
    getRowData(i, real_data);

    // Set up a random number generator with a constant seed for reproducibility
    static mt19937 gen(42); // Constant seed
    static normal_distribution<double> dist_baro(0.0, sqrt(varBaro));
    static normal_distribution<double> dist_acc(0.0, sqrt(varAccelerometer));

    // Add Gaussian noise to data[0] (position)
    (*ground)(0) = real_data[1];
    (*ground)(1) = real_data[2];
    (*ground)(2) = real_data[3];
    double noisyPosition = (*ground)(0) + dist_baro(gen);
    double noisyAccelerometer = (*ground)(2) + dist_acc(gen);


    measurements[0] = noisyPosition;
    measurements[1] = noisyAccelerometer;
    //cout << measurements[1] << endl;

    return measurements;
}

double calculateDeltaT(int i) {
    double real_data[4];
    if (!getRowData(i, real_data)) {
        cerr << "Error: Could not read time data from config.csv" << endl;
        return 0.0;
    }

    current_time = real_data[0];
    if (i == 0) {
        prev_time = current_time;
        return 0.001;
    }

    double delta_t = current_time - prev_time;
    prev_time = current_time;

    return delta_t;
}

void save_vector_to_csv(const VectorXd& x, const string& filename, bool is_measurement = false) {
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
        if (is_measurement) {
            // For measurements, save the value and two zeros
            file << x[0] << ",0,0" << endl;
        } else {
            // Save the state vector normally
            for (int i = 0; i < x.size(); ++i) {
                file << x[i];  // Write the value of each element
                if (i < x.size() - 1) {
                    file << ",";  // Separate elements by a comma
                }
            }
            file << endl;  // Add a new line after the row
        }
        file.close();  // Close the file
    } else {
        cerr << "Error: Could not open the file!" << endl;
    }
}

int getNumberOfRows(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open " << filename << endl;
        return 0;
    }

    int rows = 0;
    string line;
    while (getline(file, line)) {
        rows++;
    }
    file.close();
    return rows;
}

// Define the state transition function
VectorXd fx(const VectorXd &x, double dt) {
    VectorXd x_new(3);  // New state vector [position, velocity, acceleration]
    double velocity = x(1);

    double dragForce = 0.5 * Cd * rho * A * velocity * velocity * (velocity > 0 ? 1 : -1);
    
    x_new(0) = x(0) + x(1) * dt + 0.5 * x(2) * dt * dt;
    
    x_new(1) = x(1) + x(2) * dt;
    
    x_new(2) = x(2);


    x_new(2) = gravity - (dragForce / mass);
    x_new(1) = velocity + (x_new(2) * dt);
    x_new(0) = x(0) + (velocity * dt);
    
    return x_new;
/*     MatrixXd F(x_dim,x_dim);
    F << 1, dt, 0.5*dt*dt,
    0, 1, dt,
    0, 0, 1;
    return F * x; */
}

// Define the measurement function
VectorXd hx(const VectorXd& x) {
    MatrixXd H(z_dim, x_dim);
    H << 1, 0, 0,
         0, 0, 1;
    return  H * x;
}

int main() {
    loadFileIntoMemory(real_filename);
    VectorXd ground_state(x_dim);
    double process_variance_pos = 250.0;
    double process_variance_vel = 5.0;
    double process_variance_acc = 10.0;
    double set_pos_measurement_variance = 5000.;
    double set_acc_measurement_variance = 100.;
    VectorXd initial_state(x_dim);
    MatrixXd initial_covariance(x_dim,x_dim);
    initial_state << 0, 1, 0;
    initial_covariance << 500,   0, 0,
                            0, 500, 0,
                            0,   0, 500;

    cout << "Starting Kalman Filter simulation..." << endl;


    double alpha = 10.5, beta = 155.0, kappa = 0.55;
    MerweScaledSigmaPoints sigmaPoints(x_dim, alpha, beta, kappa);
    //Sigma points parameters 

    UKF kf(x_dim, z_dim, u_dim, fx, hx, sigmaPoints);    // Creating the object with default dimensions

 
    MatrixXd F(x_dim,x_dim), Q(x_dim,x_dim), R(z_dim,z_dim), H(z_dim, x_dim);

    H << 1, 0, 0,
         0, 0, 1;

    R << set_pos_measurement_variance, 0,
          0, set_acc_measurement_variance;

    Q << process_variance_pos, 0, 0,
         0, process_variance_vel, 0,
         0, 0, process_variance_acc;

    kf.set_Q(Q);
    kf.set_H(H);
    //kf.set_Q_discrete_white_noise(x_dim, dt, process_variance);
    kf.initialize_state(initial_state, initial_covariance);
    kf.set_R(R);
    VectorXd z(z_dim);

    int total_iterations = getNumberOfRows(real_filename);
    while (iteration < total_iterations) {
        // Update delta time
        dt = calculateDeltaT(iteration);
        dt_sum += dt;
        if(dt_sum >= kf_delay) {
            F << 1, dt_sum, 0.5*dt_sum*dt_sum,
            0, 1, dt_sum,
            0, 0, 1;    
            kf.set_F(F);

            //PREDICT
            kf.predict(dt_sum);
            //UPDATE
            // Read the sensor measurement and update the filter
            double* dataMeasured = readSensor(iteration, set_pos_measurement_variance, set_acc_measurement_variance, &ground_state);
            z(0) = dataMeasured[0];
            z(1) = dataMeasured[1];
            kf.update(z);

            // Save the results
            // Create a new measurement vector z_saved (dimension 3)
            VectorXd z_saved(x_dim);
            VectorXd state_saved(x_dim+1);
            z_saved << z(0), 0, z(1);  // Only position is sensed, velocity and acceleration are unknown
            state_saved << current_time, (kf.get_x())(0), (kf.get_x())(1), (kf.get_x())(2);  
            save_vector_to_csv(z_saved, "measurements.csv");
            save_vector_to_csv(state_saved, "output.csv");

            //Update NEES
            kf.NEES_UPDATE(ground_state);

            //Save the covariances for filter evaluation
            VectorXd results_evaluation(x_dim * 2);
            results_evaluation(0) = (kf.get_real_error())(0);
            results_evaluation(1) = (kf.get_P())(0,0);
            results_evaluation(2) = (kf.get_real_error())(1);
            results_evaluation(3) = (kf.get_P())(1,1);
            results_evaluation(4) = (kf.get_real_error())(2);
            results_evaluation(5) = (kf.get_P())(2,2);
            save_vector_to_csv(results_evaluation, "evaluation.csv");
            dt_sum = 0;
        }
        iteration++;
    }

    cout << "NESS Average: " << kf.get_NEES_AVG() << endl;
    
    return 0;
}