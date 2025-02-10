#include "Sigma.h"
#include <ArduinoEigen.h>

using namespace Eigen;

//Defualt constructor:
MerweScaledSigmaPoints::MerweScaledSigmaPoints() {
  // Provide default values or set them to zero
  this->n = 0;
  this->alpha = 0;
  this->beta = 0;
  this->kappa = 0;
  this->sigmas = MatrixXf::Zero(n, 2 * n + 1);
  // Optionally, computeWeights() or leave it for later
}


// Constructor
MerweScaledSigmaPoints::MerweScaledSigmaPoints(int dim, float a, float b, float k) {
  this->n = dim;
  this->alpha = a;
  this->beta = b;
  this->kappa = k;
  this->sigmas = MatrixXf::Zero(n, 2 * n + 1);
  // Compute weights
  computeWeights();
}

VectorXf MerweScaledSigmaPoints::getSigmaPoint(int i) const {
  return sigmas.col(i);
}

MatrixXf MerweScaledSigmaPoints::getSigmaPoints() const {
  return sigmas;
}

void MerweScaledSigmaPoints::setSigmaPoint(int i, VectorXf point) {
  sigmas.col(i) = point;
}

// Number of sigma points
int MerweScaledSigmaPoints::numSigmas() const {
  return 2 * n + 1;
}

// Compute sigma points
MatrixXf MerweScaledSigmaPoints::sigmaPoints(const VectorXf& x, const MatrixXf& P) {
  float lambda = (alpha * alpha * (n + kappa)) - n;
  MatrixXf U = cholesky((lambda + n) * P);

  sigmas.setZero();  // Ensure it's zeroed out before writing to it

  sigmas.col(0) = x;  // Direct assignment, The first sigma point is just the mean
  // Print dimensions of sigmas, x, and U
  for (int k = 0; k < n; ++k) {
    sigmas.col(k + 1) = x - U.row(k).transpose();     // Check the sources for this equations
    sigmas.col(n + k + 1) = x + U.row(k).transpose();  // N Check the sources for this equations
  }

  return sigmas;
}

// Helper function to compute the Cholesky decomposition (default sqrt method)
MatrixXf MerweScaledSigmaPoints::cholesky(const MatrixXf& P) {
  LLT<MatrixXf> llt(P);
  if (llt.info() == Success) {
    return llt.matrixL();
  } else {
    // Handle failure (e.g., return identity matrix)
    return MatrixXf::Identity(P.rows(), P.cols());
  }
}

// Getter for weights
const VectorXf& MerweScaledSigmaPoints::getWeightsMean() const {
  return Wm;
}

const VectorXf& MerweScaledSigmaPoints::getWeightsCovariance() const {
  return Wc;
}

// Print object information
void MerweScaledSigmaPoints::print() const {
  Serial.println("MerweScaledSigmaPoints object");
  Serial.print("n: ");
  Serial.println(n);
  
  Serial.print("alpha: ");
  Serial.println(alpha, 6);  // Print with 6 decimal places
  
  Serial.print("beta: ");
  Serial.println(beta, 6);
  
  Serial.print("kappa: ");
  Serial.println(kappa, 6);

  Serial.print("Wm: ");
  for (int i = 0; i < Wm.size(); i++) {
    Serial.print(Wm(i), 6);  // Print each weight with 6 decimal places
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Wc: ");
  for (int i = 0; i < Wc.size(); i++) {
    Serial.print(Wc(i), 6);
    Serial.print(" ");
  }
  Serial.println();
}

// Compute weights
void MerweScaledSigmaPoints::computeWeights() {
  // Correct calculation of lambda
  float lambda = alpha * alpha * (n + kappa) - n;

  // Compute the common weight for all sigma points except the first
  float c = 0.5 / (n + lambda);

  // Initialize weights
  Wm = VectorXf::Constant(2 * n + 1, c);
  Wc = VectorXf::Constant(2 * n + 1, c);

  // Set the first weight for the mean and covariance
  Wm(0) = lambda / (n + lambda);
  Wc(0) = lambda / (n + lambda) + (1 - alpha * alpha + beta);
}