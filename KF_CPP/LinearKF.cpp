#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <iomanip>
#include "LinearKF.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std; 

// Constructor
LinearKF::LinearKF(int x_dim, int z_dim, int u_dim)
    : x(VectorXd::Zero(x_dim)), 
      P(MatrixXd::Identity(x_dim, x_dim)),
      F(MatrixXd::Identity(x_dim, x_dim)),
      B(MatrixXd::Zero(x_dim, x_dim)),
      H(MatrixXd::Zero(z_dim, x_dim)),
      R(MatrixXd::Identity(z_dim, z_dim)),
      Q(MatrixXd::Identity(x_dim, x_dim)),
      K(MatrixXd::Zero(x_dim, z_dim)),
      _I(MatrixXd::Identity(x_dim, x_dim)),
      _IKH(MatrixXd::Identity(x_dim, x_dim))
    
    //Memeber variables initialized. Better practice than using this->x... 
{   
    this->x_dim = x_dim;
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
LinearKF::~LinearKF() {}

// Initialize the filter with necessary parameters
void LinearKF::initialize_state(VectorXd initial_state, MatrixXd initial_covariance) {
    set_x(initial_state);
    set_P(initial_covariance);
}

// Predict the next state
void LinearKF::predict(VectorXd u) {
    if (u.size() == 0) {
        u = VectorXd::Zero(x_dim);  // In case we are not using u so nothing breaks
    }
    x = F * x + B * u;
    P = F * P * (F.transpose()) + Q;
}


// Update the state with a new measurement
void LinearKF::update(VectorXd z) {
    K = (P*H.transpose())*((H*(P*H.transpose()) + R).inverse());
    x = x + K*(z - (H * x));
    _IKH = (_I-(K*H));
    P = ((_IKH*P)*_IKH.transpose()) + ((K*R)*K.transpose());
}

//GET-SET x
VectorXd LinearKF::get_x() const {
    return x;
}

void LinearKF::set_x(VectorXd x) {
    this->x = x;
}

//GET-SET P
MatrixXd LinearKF::get_P() const {
    return P;
}

void LinearKF::set_P(MatrixXd P) {
    this->P = P;
}

//GET-SET F
MatrixXd LinearKF::get_F() const {
    return F;
}

void LinearKF::set_F(MatrixXd F) {
    this->F = F;
}

//GET-SET Q
MatrixXd LinearKF::get_Q() const {
    return Q;
}

void LinearKF::set_Q(MatrixXd Q) {
    this->Q = Q;
}

MatrixXd LinearKF::set_Q_discrete_white_noise(int dim, double dt, double var) {
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
MatrixXd LinearKF::get_H() const {
    return H;
}

void LinearKF::set_H(MatrixXd H) {
    this->H = H;
}

//GET-SET R
MatrixXd LinearKF::get_R() const {
    return R;
}

void LinearKF::set_R(MatrixXd R) {
    this->R = R;
}

//GET-SET B
MatrixXd LinearKF::get_B() const {
    return B;
}

void LinearKF::set_B(MatrixXd B) {
    this->B = B;
}

//GET K
MatrixXd LinearKF::get_K() const {
    return K;
}
