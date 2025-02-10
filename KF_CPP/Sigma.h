#ifndef SIGMA_H
#define SIGMA_H

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class MerweScaledSigmaPoints {
public:
    // Constructor with raw function pointers
    MerweScaledSigmaPoints(int n, double alpha, double beta, double kappa,
                           MatrixXd (*sqrt_method)(const MatrixXd&) = nullptr,
                           VectorXd (*subtract)(const VectorXd&, const VectorXd&) = nullptr);

    // Number of sigma points
    int numSigmas() const;

    // Compute sigma points
    MatrixXd sigmaPoints(const VectorXd& x, const MatrixXd& P);

    // Getter for Sigma Points
    VectorXd getSigmaPoint(int i) const;
    MatrixXd getSigmaPoints() const;
    void setSigmaPoint(int i, VectorXd point);
    

    // Getter for weights
    const VectorXd& getWeightsMean() const;
    const VectorXd& getWeightsCovariance() const;

    // Print object information
    void print() const;

private:
    int n; // Dimensionality of the state
    double alpha, beta, kappa; // Scaling parameters
    VectorXd Wm, Wc; // Weights for mean and covariance
    MatrixXd (*sqrt_method)(const MatrixXd&); // Square root method
    VectorXd (*subtract)(const VectorXd&, const VectorXd&); // Subtract function
    MatrixXd sigmas;  // Store sigma points as a member variable

    // Compute weights
    void computeWeights();
};

#endif // SIGMA_H