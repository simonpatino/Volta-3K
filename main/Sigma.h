#ifndef SIGMA_H
#define SIGMA_H

#include <ArduinoEigen.h>

using namespace Eigen;

class MerweScaledSigmaPoints {
public:
  // Constructors function pointers
  MerweScaledSigmaPoints(); //default
  MerweScaledSigmaPoints(int dim, float a, float b, float k);

  // Number of sigma points
  int numSigmas() const;

  // Compute sigma points
  MatrixXf sigmaPoints(const VectorXf& x, const MatrixXf& P);

  // Getter for Sigma Points
  VectorXf getSigmaPoint(int i) const;
  MatrixXf getSigmaPoints() const;
  void setSigmaPoint(int i, VectorXf point);


  // Getter for weights
  const VectorXf& getWeightsMean() const;
  const VectorXf& getWeightsCovariance() const;

  // Print object information
  void print() const;

private:
  int n;                                                   // Dimensionality of the state
  float alpha, beta, kappa;                               // Scaling parameters
  VectorXf Wm, Wc;                                         // Weights for mean and covariance
  MatrixXf sigmas;                                         // Store sigma points as a member variable
  MatrixXf cholesky(const MatrixXf& P);
  // Compute weights
  void computeWeights();
};

#endif  // SIGMA_H