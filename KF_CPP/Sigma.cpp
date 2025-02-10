#include "Sigma.h"
#include <iostream>

using namespace std;
using namespace Eigen;


// Helper function to compute the Cholesky decomposition (default sqrt method)
MatrixXd cholesky(const MatrixXd& P) {
    LLT<MatrixXd> llt(P);
    if (llt.info() == Success) {
        return llt.matrixL();
    } else {
        // Handle failure (e.g., return identity matrix)
        return MatrixXd::Identity(P.rows(), P.cols());
    }
}

VectorXd defaultSubtract(const VectorXd& x, const VectorXd& y) {
    return x - y;
}

// Constructor
MerweScaledSigmaPoints::MerweScaledSigmaPoints(int n, double alpha, double beta, double kappa,
                           MatrixXd (*sqrt_method)(const MatrixXd&),
                           VectorXd (*subtract)(const VectorXd&, const VectorXd&))
    : n(n), alpha(alpha), beta(beta), kappa(kappa), sigmas(MatrixXd::Zero(n, 2*n+1)) {
    // Set default sqrt method if not provided
    if (sqrt_method == nullptr) {
        this->sqrt_method = cholesky; // Use a default function
    } else {
        this->sqrt_method = sqrt_method; // Use the provided function
    }

    // Set default subtract method if not provided
    if (subtract == nullptr) {
        this->subtract = defaultSubtract; // Use the default function
    } else {
        this->subtract = subtract; // Use the provided function
    }

    // Compute weights
    computeWeights();
}

VectorXd MerweScaledSigmaPoints::getSigmaPoint(int i) const {
    // Ensure that the column index is valid
    if (i < 0 || i >= sigmas.cols()) {
        std::cerr << "Column index out of bounds!" << std::endl;
        exit(1); // Exit with error
    }

    // Return the i-th col as a vector
    return sigmas.col(i);
}

MatrixXd MerweScaledSigmaPoints::getSigmaPoints() const {
    return sigmas;
}

void MerweScaledSigmaPoints::setSigmaPoint(int i, VectorXd point) {
    sigmas.col(i) = point;
}

// Number of sigma points
int MerweScaledSigmaPoints::numSigmas() const {
    return 2 * n + 1;
}

// Compute sigma points
MatrixXd MerweScaledSigmaPoints::sigmaPoints(const VectorXd& x, const MatrixXd& P) {
    if (x.size() != n) {
        cerr << "Error: Size of x does not match n." << endl;
        return MatrixXd();
    }

    double lambda = (alpha * alpha * (n + kappa)) - n;
    MatrixXd U = sqrt_method((lambda + n) * P);

    sigmas.setZero();  // Ensure it's zeroed out before writing to it

    sigmas.col(0) = x;  // Direct assignment, The first sigma point is just the mean
    for (int k = 0; k < n; ++k) {
        sigmas.col(k + 1) = subtract(x, -U.row(k));  // Check the sources for this equations
        sigmas.col(n + k + 1) = subtract(x, U.row(k));  // N Check the sources for this equations
    }

    return sigmas;
}

// Getter for weights
const VectorXd& MerweScaledSigmaPoints::getWeightsMean() const {
    return Wm;
}

const VectorXd& MerweScaledSigmaPoints::getWeightsCovariance() const {
    return Wc;
}

// Print object information
void MerweScaledSigmaPoints::print() const {
    cout << "MerweScaledSigmaPoints object" << endl;
    cout << "n: " << n << endl;
    cout << "alpha: " << alpha << endl;
    cout << "beta: " << beta << endl;
    cout << "kappa: " << kappa << endl;
    cout << "Wm: " << Wm.transpose() << endl;
    cout << "Wc: " << Wc.transpose() << endl;
}

// Compute weights
void MerweScaledSigmaPoints::computeWeights() {
    // Correct calculation of lambda
    double lambda = alpha * alpha * (n + kappa) - n;

    // Compute the common weight for all sigma points except the first
    double c = 0.5 / (n + lambda);

    // Initialize weights
    Wm = VectorXd::Constant(2 * n + 1, c);
    Wc = VectorXd::Constant(2 * n + 1, c);

    // Set the first weight for the mean and covariance
    Wm(0) = lambda / (n + lambda);
    Wc(0) = lambda / (n + lambda) + (1 - alpha * alpha + beta);
}