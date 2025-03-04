#include <ArduinoEigen.h>
#include "KalmanFilter.h"
#include "Sigma.h"

// Constructor for Arduino
KalmanFilter::KalmanFilter(int x_dim, int z_dim, int u_dim, const MerweScaledSigmaPoints& sigma_points)
  : isValid(true)  // Initialize the validity flag
{
  // Assign dimensions
  this->x_dim = x_dim;
  this->z_dim = z_dim;
  this->u_dim = u_dim;
  this->points = sigma_points;

  // Validate dimensions
  if (x_dim < 1 || z_dim < 1 || u_dim < 0) {
    isValid = false;  // Mark as invalid
    Serial.println("Error: Invalid dimensions provided to KalmanFilter constructor.");
    return;  // Exit early
  }

  // Initialize matrices and vectors
  x = VectorXf::Zero(x_dim);
  y = VectorXf::Zero(z_dim);
  y_av = VectorXf::Zero(z_dim);
  P = MatrixXf::Identity(x_dim, x_dim);
  F = MatrixXf::Identity(x_dim, x_dim);
  B = MatrixXf::Zero(x_dim, x_dim);
  H = MatrixXf::Zero(z_dim, x_dim);
  R = MatrixXf::Identity(z_dim, z_dim);
  Q = MatrixXf::Identity(x_dim, x_dim);
  K = MatrixXf::Zero(x_dim, z_dim);
  _I = MatrixXf::Identity(x_dim, x_dim);
  _IKH = MatrixXf::Identity(x_dim, x_dim);
  S = MatrixXf::Zero(z_dim, z_dim);
  y_real = VectorXf::Zero(x_dim);
}



// Destructor
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::initialize_state(VectorXf initial_state, MatrixXf initial_covariance) {
  set_x(initial_state);
  set_P(initial_covariance);
  it = 0;
  NEES_avg = 0;
}

// Predict the next state
void KalmanFilter::predict(float dt, VectorXf u) {
  if (u.size() == 0) {
    u = VectorXf::Zero(x_dim);  // In case we are not using u so nothing breaks
  }

  // Calculate the sigma points for the given mean and covariance
  points.sigmaPoints(this->x, this->P);
  // Predict the sigma points using the state transition function (fx)
  // You will apply fx (state transition function) to each sigma point
  for (int i = 0; i < points.numSigmas(); ++i) {
    points.setSigmaPoint(i, fx(points.getSigmaPoint(i), dt));
  }

  // Use the unscented transform to compute the predicted state (x) and covariance (P)
  unscented_transform(points);
}


// Update the state with a new measurement
void KalmanFilter::update(VectorXf z) {
  S = H * (P * H.transpose()) + R;
  K = (P * H.transpose()) * (S.inverse());
  y = z - (H * x);

  // Compute likelihood
  float exponent = -0.5 * y.transpose() * S.ldlt().solve(y); // Robust exponent calculation
  float log_normalization = -0.5 * (z.size() * log(2 * M_PI) + S.ldlt().vectorD().array().log().sum()); // Log normalization
  log_likelihood = log_normalization + exponent; // Log-likelihood
  likelihood = exp(log_likelihood); // Likelihood

  if(!(abs(log_likelihood) > rejectionThreshold)) { //Measurment rejection if the log_likelihood signals a very unlikely measurement
    x = x + K * y;
    _IKH = (_I - (K * H));
    P = ((_IKH * P) * _IKH.transpose()) + ((K * R) * K.transpose());
  }
}

void KalmanFilter::unscented_transform(const MerweScaledSigmaPoints& sigma_points) {
  // Compute mean
  MatrixXf P_temp = MatrixXf::Zero(x_dim, x_dim);
  x = sigma_points.getSigmaPoints() * sigma_points.getWeightsMean();  // Simple weighted sum if no function is provided


  for (int i = 0; i < sigma_points.numSigmas(); ++i) {
    VectorXf diff = sigma_points.getSigmaPoint(i) - x;                        // (n x 1)
    P_temp += sigma_points.getWeightsCovariance()(i) * (diff * diff.transpose());  // (n x n)
  }
  // Add noise covariance if provided
  P_temp += Q;

  P = P_temp;
}

//GET-SET x
VectorXf KalmanFilter::get_x() const {
  return x;
}

void KalmanFilter::set_x(VectorXf x) {
  this->x = x;
}

//GET y and y_av
VectorXf KalmanFilter::get_y_av() const {
  return y_av;
}

VectorXf KalmanFilter::get_y() const {
  return y;
}

//GET-SET P
MatrixXf KalmanFilter::get_P() const {
  return P;
}

void KalmanFilter::set_P(MatrixXf P) {
  this->P = P;
}

//GET-SET F
MatrixXf KalmanFilter::get_F() const {
  return F;
}

void KalmanFilter::set_F(MatrixXf F) {
  this->F = F;
}

void KalmanFilter::set_fx(StateTransitionFunc f) {
  this->fx = f;
}

//GET-SET Q
MatrixXf KalmanFilter::get_Q() const {
  return Q;
}

void KalmanFilter::set_Q(MatrixXf Q) {
  this->Q = Q;
}

MatrixXf KalmanFilter::set_Q_discrete_white_noise(int dim, float dt, float var) {
  // Check if dimension is valid
  if (dim < 2 || dim > 4) {
    Serial.println("dim must be between 2 and 4");
  }

  // Create the Q matrix as a dynamic matrix (depending on dimension)
  MatrixXf Q;

  if (dim == 2) {
    Q = MatrixXf(2, 2);
    Q << 0.25 * dt * dt * dt * dt, 0.5 * dt * dt * dt,
      0.5 * dt * dt * dt, dt * dt;
  } else if (dim == 3) {
    Q = MatrixXf(3, 3);
    Q << 0.25 * dt * dt * dt * dt, 0.5 * dt * dt * dt, 0.5 * dt * dt,
      0.5 * dt * dt * dt, dt * dt, dt,
      0.5 * dt * dt, dt, 1;
  } else {
    Q = MatrixXf(4, 4);
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
MatrixXf KalmanFilter::get_H() const {
  return H;
}

void KalmanFilter::set_H(MatrixXf H) {
  this->H = H;
}

void KalmanFilter::set_hx(MeasurementFunc h) {
  this->hx = h;
}

//GET-SET R
MatrixXf KalmanFilter::get_R() const {
  return R;
}

void KalmanFilter::set_R(MatrixXf R) {
  this->R = R;
}

//GET-SET B
MatrixXf KalmanFilter::get_B() const {
  return B;
}

void KalmanFilter::set_B(MatrixXf B) {
  this->B = B;
}

//GET K
MatrixXf KalmanFilter::get_K() const {
  return K;
}

//GET NEES
float KalmanFilter::get_NEES_AVG() const {
  return NEES_avg;
}

//GET Current NEES
float KalmanFilter::get_NEES_CURR() const {
  return currentNEES;
}

//GET Likelihoods
float KalmanFilter::get_log_likelihood() const {
  return log_likelihood;
}

float KalmanFilter::get_likelihood() const {
  return likelihood;
}

float KalmanFilter::get_rejectionThreshold() const {
  return rejectionThreshold;
}

//Get real vs estimation state error
VectorXf KalmanFilter::get_y_real() const {
  return y_real;
}

VectorXf KalmanFilter::error_update(VectorXf ground_state) {
  y_real = x - ground_state;
  return y_real;
}


float KalmanFilter::NEES_UPDATE(VectorXf ground_state) {
  error_update(ground_state);
  currentNEES = (y_real.transpose()) * (P.inverse()) * (y_real);
  NEES_avg = NEES_avg + (currentNEES - NEES_avg) / it;  // Update error average
  return currentNEES;
}


void KalmanFilter::printMatrix(const MatrixXf& matrix) {
  int rows = matrix.rows();
  int cols = matrix.cols();

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      Serial.print(matrix(i, j), 2);  // Print element with 2 decimal places
      Serial.print("\t");             // Tab separator
    }
    Serial.println();  // Move to the next line after each row
  }
}
