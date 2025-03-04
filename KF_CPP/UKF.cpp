#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include "Sigma.h"
#include "UKF.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std; 

// Constructor
UKF::UKF(int x_dim, int z_dim, int u_dim,
        StateTransitionFunc fx, MeasurementFunc hx,
        const MerweScaledSigmaPoints& sigma_points)
    : x(VectorXd::Zero(x_dim)), 
      y(VectorXd::Zero(z_dim)), 
      y_av(VectorXd::Zero(z_dim)), 
      P(MatrixXd::Identity(x_dim, x_dim)),
      F(MatrixXd::Identity(x_dim, x_dim)),
      B(MatrixXd::Zero(x_dim, x_dim)),
      H(MatrixXd::Zero(z_dim, x_dim)),
      R(MatrixXd::Identity(z_dim, z_dim)),
      Q(MatrixXd::Identity(x_dim, x_dim)),
      K(MatrixXd::Zero(x_dim, z_dim)),
      _I(MatrixXd::Identity(x_dim, x_dim)),
      _IKH(MatrixXd::Identity(x_dim, x_dim)),
      S(MatrixXd::Zero(z_dim, z_dim)),
      real_error(VectorXd::Zero(x_dim)),
      points(sigma_points)
    
    //Memeber variables initialized. Better practice than using this->x... 
{   
    this->x_dim = x_dim;
    this->fx = fx;
    this->hx = hx;
    if (x_dim < 1) {
        throw invalid_argument("dim_x must be 1 or greater");
    }
    if (z_dim < 1) {
        throw invalid_argument("dim_z must be 1 or greater");
    }
    if (u_dim < 0) {
        throw invalid_argument("dim_u must be 0 or greater");
    }
}

// Destructor
UKF::~UKF() {}

// Initialize the filter with necessary parameters
void UKF::initialize_state(VectorXd initial_state, MatrixXd initial_covariance) {
    set_x(initial_state);
    set_P(initial_covariance);
    it = 0;
    NEES_avg = 0;
}

// Predict the next state
/* void UKF::predict(double dt, VectorXd u) {
    if (u.size() == 0) {
        u = VectorXd::Zero(x_dim);  // In case we are not using u so nothing breaks
    }
    x = F * x + B * u;
    P = F * P * (F.transpose()) + Q;
} */

void UKF::predict(double dt, VectorXd u) {
    if (u.size() == 0) {
        u = VectorXd::Zero(x_dim);  // In case we are not using u so nothing breaks
    }

    // Calculate the sigma points for the given mean and covariance
    points.sigmaPoints(this->x, this->P);

    // Predict the sigma points using the state transition function (fx)
    // You will apply fx (state transition function) to each sigma point
    for (int i = 0; i < points.numSigmas(); ++i) {
        points.setSigmaPoint(i, fx(points.getSigmaPoint(i), dt));
    }

    //cout << points.getSigmaPoints() <<endl;

    // Use the unscented transform to compute the predicted state (x) and covariance (P)
    pair<VectorXd, MatrixXd> result = unscented_transform(points);
    this->x = result.first;
    this->P = result.second;

    // Update sigma points to reflect the new predicted state and covariance
    //points.sigmaPoints(this->x, this->P);  
}

// Function signatures for mean and residual functions
typedef VectorXd (*MeanFn)(const MatrixXd&, const VectorXd&);

// The unscented transform function
pair<VectorXd, MatrixXd> UKF::unscented_transform(const MerweScaledSigmaPoints& sigma_points) 
{
    // Compute mean
/*     cout << "Sigma points" << endl;
    cout << sigma_points.getSigmaPoints() << endl;
    cout << "Weights" << endl;
    cout << sigma_points.getWeightsMean() << endl << endl; */

    VectorXd x_mean(x_dim);
    MatrixXd P_temp = MatrixXd::Zero(x_dim, x_dim);


    x_mean =  sigma_points.getSigmaPoints() * sigma_points.getWeightsMean();  // Simple weighted sum if no function is provided
    
/*     cout << "Mean   " << endl;
    cout << x_mean << endl << endl; */
    // Standard residual computation

    for (int i = 0; i < sigma_points.numSigmas(); ++i) {
        VectorXd diff = sigma_points.getSigmaPoint(i) - x_mean;  // (n x 1)
        P_temp += sigma_points.getWeightsCovariance()(i) * (diff * diff.transpose());    // (n x n)
    }
    // Add noise covariance if provided
    P_temp += Q;

    return make_pair(x_mean, P_temp);
}


// Update the state with a new measurement
void UKF::update(VectorXd z) {
    S = H*(P*H.transpose()) + R;
    K = (P*H.transpose())*(S.inverse());
    y = z - (H * x);
    x = x + K*y;
    _IKH = (_I-(K*H));
    P = ((_IKH*P)*_IKH.transpose()) + ((K*R)*K.transpose());

    it++;  // Increment iteration count
    y_av = y_av + (y - y_av) / it;  // Update error average

    // Compute likelihood
    double exponent = -0.5 * y.transpose() * S.ldlt().solve(y); // Robust exponent calculation
    double log_normalization = -0.5 * (z.size() * log(2 * M_PI) + S.ldlt().vectorD().array().log().sum()); // Log normalization
    log_likelihood = log_normalization + exponent; // Log-likelihood
    likelihood = exp(log_likelihood); // Likelihood

    // Output likelihood (or log-likelihood for debugging)
    /*cout << "Likelihood: " << likelihood << endl; */
    /* cout << likelihood << endl; */
}

//GET-SET x
VectorXd UKF::get_x() const {
    return x;
}

void UKF::set_x(VectorXd x) {
    this->x = x;
}


//GET y and y_av
VectorXd UKF::get_y_av() const {
    return y_av;
}

VectorXd UKF::get_y() const {
    return y;
}

//GET-SET P
MatrixXd UKF::get_P() const {
    return P;
}

void UKF::set_P(MatrixXd P) {
    this->P = P;
}

//GET-SET F
MatrixXd UKF::get_F() const {
    return F;
}

void UKF::set_F(MatrixXd F) {
    this->F = F;
}

//GET-SET Q
MatrixXd UKF::get_Q() const {
    return Q;
}

void UKF::set_Q(MatrixXd Q) {
    this->Q = Q;
}

MatrixXd UKF::set_Q_discrete_white_noise(int dim, double dt, double var) {
    // Check if dimension is valid
    if (dim < 2 || dim > 4) {
        throw invalid_argument("dim must be between 2 and 4");
    }

    // Create the Q matrix as a dynamic matrix (depending on dimension)
    MatrixXd Q;

    if (dim == 2) {
        Q = MatrixXd(2, 2);
        Q << 0.25 * dt * dt * dt * dt, 0.5 * dt * dt * dt,
             0.5 * dt * dt * dt, dt * dt;
    } else if (dim == 3) {
        Q = MatrixXd(3, 3);
        Q << 0.25 * dt * dt * dt * dt, 0.5 * dt * dt * dt, 0.5 * dt * dt,
             0.5 * dt * dt * dt, dt * dt, dt,
             0.5 * dt * dt, dt, 1;
    } else {
        Q = MatrixXd(4, 4);
        Q << (dt * dt * dt * dt * dt * dt) / 36, (dt * dt * dt * dt * dt) / 12, (dt * dt * dt * dt) / 6, (dt * dt * dt) / 6,
             (dt * dt * dt * dt * dt) / 12, (dt * dt * dt * dt) / 4, (dt * dt * dt) / 2, (dt * dt) / 2,
             (dt * dt * dt * dt) / 6, (dt * dt * dt) / 2, dt * dt, dt,
             (dt * dt * dt) / 6, (dt * dt) / 2, dt, 1;
    }

    // Scale by variance
    Q *= var;

    this->Q = Q;
    return Q;
}

//GET-SET H
MatrixXd UKF::get_H() const {
    return H;
}

void UKF::set_H(MatrixXd H) {
    this->H = H;
}

//GET-SET R
MatrixXd UKF::get_R() const {
    return R;
}

void UKF::set_R(MatrixXd R) {
    this->R = R;
}

//GET-SET B
MatrixXd UKF::get_B() const {
    return B;
}

void UKF::set_B(MatrixXd B) {
    this->B = B;
}

//GET K
MatrixXd UKF::get_K() const {
    return K;
}
//GET NEES
double UKF::get_NEES_AVG() const {
    return NEES_avg;
}

//GET Likelihoods
double UKF::get_log_likelihood() const {
    return log_likelihood;
}

double UKF::get_likelihood() const {
    return likelihood;
}

//Get real vs estimation state error
VectorXd UKF::get_real_error() const {
    return real_error;
}

VectorXd UKF::error_update(VectorXd& ground_state) {
    real_error = x - ground_state;
    return real_error;
}


double UKF::NEES_UPDATE(VectorXd& ground_state) {
    error_update(ground_state);
    double current_NEES = (real_error.transpose())*(P.inverse())*(real_error);
    NEES_avg = NEES_avg + (current_NEES - NEES_avg) / it;  // Update error average
    return current_NEES;
}