#include "Sigma.h"
#include <iostream>

//Compile:
// g++ -std=c++11 -I/usr/include/eigen3 sigma_tester.cpp Sigma.cpp -o merwe_example && ./merwe_example

using namespace std;

int main() {
    int n = 3;
    double alpha = .1, beta = 2.0, kappa = 1.;

    MerweScaledSigmaPoints points(n, alpha, beta, kappa);

    VectorXd x(n);
    x << 5, 2, 1;
    MatrixXd P = MatrixXd::Identity(n, n) * 9;

    MatrixXd sigmas = points.sigmaPoints(x, P);
    cout << "Sigma Points:\n" << sigmas << endl;

    points.print();

    cout << points.getWeightsMean().rows() << endl;
    return 0;
}