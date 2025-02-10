#ifndef KF_H
#define KF_H

#include <ArduinoEigen.h>
#include "Sigma.h"

using namespace Eigen;

class KalmanFilter {
public:
  // Function pointer types for fx and hx
  using StateTransitionFunc = VectorXf (*)(const VectorXf&, float);
  using MeasurementFunc = VectorXf (*)(const VectorXf&);
  void printMatrix(const MatrixXf& matrix);
  // Constructor
  KalmanFilter(int x_dim, int z_dim, int u_dim, const MerweScaledSigmaPoints& sigma_points);
  bool isValid;  // Flag to indicate if the filter was initialized successfully

  // Destructor
  ~KalmanFilter();

  // Initialize the filter with necessary parameters
  void initialize_state(VectorXf initial_state, MatrixXf initial_covariance);

  // Predict the next state
  void predict(float dt, VectorXf u = VectorXf::Zero(0));

  void unscented_transform(const MerweScaledSigmaPoints& sigma_points);

  // Update the state with a new measurement
  void update(VectorXf z);

  //Get the current state estimate
  VectorXf get_x() const;
  void set_x(VectorXf x);

  VectorXf get_y_av() const;
  VectorXf get_y() const;

  //Get-set state covariance P
  MatrixXf get_P() const;
  void set_P(MatrixXf P);

  //Get-set state transition F
  MatrixXf get_F() const;
  void set_F(MatrixXf F);
  void set_fx(StateTransitionFunc fx);

  //Get-set process noise Q
  MatrixXf get_Q() const;
  void set_Q(MatrixXf Q);
  MatrixXf set_Q_discrete_white_noise(int dim, float dt = 1.0, float var = 1.0);

  //Get-set process noise H
  MatrixXf get_H() const;
  void set_H(MatrixXf H);
  void set_hx(MeasurementFunc h);

  //Get-set measurement noise R
  MatrixXf get_R() const;
  void set_R(MatrixXf R);

  //Get-set control input B
  MatrixXf get_B() const;
  void set_B(MatrixXf B);

  //Get Kalman gain
  MatrixXf get_K() const;

  //Get NESS AVERAGE
  float get_NEES_AVG() const;

  //Get Likelihoods
  float get_likelihood() const;
  float get_log_likelihood() const;

  //Get real-estimated error
  VectorXf get_y_real() const;

  //Update the new NEES average if the current run is a simulation
  float NEES_UPDATE(VectorXf ground_state);

  //Update the real estimated state error
  VectorXf error_update(VectorXf ground_state);

private:
  //TODO: Change into matrix form
  VectorXf x;              // Current state estimate
  VectorXf y;              // Current error between prediction and measurement
  VectorXf y_av;           // Average error between prediction and measurement
  MatrixXf P;              // Estimate covariance
  MatrixXf F;              // Transition function
  StateTransitionFunc fx;  // State transition function
  MatrixXf B;              // Control input matrix
  MatrixXf H;              // Measurement transition
  MeasurementFunc hx;      // Measurement function
  MatrixXf R;              // Measurement noise covariance
  MatrixXf Q;              // Process noise covariance
  MatrixXf K;              // Kalman gain
  MatrixXf _I;             //Identity used to update covariance (see Kalman Filter equations)
  MatrixXf _IKH;           //Identity used to update covariance (see Kalman Filter equations)
  MatrixXf S;              //Identity used to update covariance (see Kalman Filter equations)
  VectorXf y_real;         //For simulation only

  int x_dim, z_dim, u_dim, it;  //'it' is iteration number
  float NEES_avg, log_likelihood, likelihood;
  MerweScaledSigmaPoints points;  // Add this member variable

};

#endif