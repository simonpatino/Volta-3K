#ifndef LINEARKF_H
#define LINEARKF_H

#include <Eigen/Dense>

using namespace Eigen;

class LinearKF {
public:
    // Constructor
    LinearKF(int x_dim, int z_dim, int u_dim);

    // Destructor
    ~LinearKF();

    // Initialize the filter with necessary parameters
    void initialize_state(VectorXd initial_state, MatrixXd initial_covariance);

    // Predict the next state
    void predict(VectorXd u = VectorXd::Zero(0));

    // Update the state with a new measurement
    void update(VectorXd z);

    //Get the current state estimate
    VectorXd get_x() const;
    void set_x(VectorXd x);

    VectorXd get_y_av() const;
    VectorXd get_y() const;

    //Get-set state covariance P
    MatrixXd get_P() const;
    void set_P(MatrixXd P);

    //Get-set state transition F
    MatrixXd get_F() const;
    void set_F(MatrixXd F);

    //Get-set process noise Q
    MatrixXd get_Q() const;
    void set_Q(MatrixXd Q);
    MatrixXd set_Q_discrete_white_noise(int dim, double dt = 1.0, double var = 1.0);

    //Get-set process noise H
    MatrixXd get_H() const;
    void set_H(MatrixXd H);

    //Get-set measurement noise R
    MatrixXd get_R() const;
    void set_R(MatrixXd R);

    //Get-set control input B 
    MatrixXd get_B() const;
    void set_B(MatrixXd B);

    //Get Kalman gain
    MatrixXd get_K() const;


    //Get NESS AVERAGE
    double get_NEES_AVG() const;

    //Get Likelihoods
    double get_likelihood() const;
    double get_log_likelihood() const;

    //Get real-estimated error
    VectorXd get_real_error() const;

    //Update the new NEES average if the current run is a simulation
    double NEES_UPDATE(VectorXd ground_state);

    //Update the real estimated state error
    VectorXd error_update(VectorXd ground_state);

private:
    //TODO: Change into matrix form
    VectorXd x;  // Current state estimate
    VectorXd y;  // Current error between prediction and measurement
    VectorXd y_av;  // Average error between prediction and measurement
    MatrixXd P;  // Estimate covariance
    MatrixXd F;  // Transition function
    MatrixXd B;  // Control input matrix
    MatrixXd H; // Measurement transition 
    MatrixXd R;  // Measurement noise covariance
    MatrixXd Q; // Process noise covariance
    MatrixXd K;  // Kalman gain
    MatrixXd _I; //Identity used to update covariance (see Kalman Filter equations)
    MatrixXd _IKH; //Identity used to update covariance (see Kalman Filter equations)
    MatrixXd S; //Identity used to update covariance (see Kalman Filter equations)
    VectorXd real_error;
    double NEES_avg;
    int x_dim, it; //it is iteration number
    double log_likelihood, likelihood;
};

#endif // LINEARKF_H
