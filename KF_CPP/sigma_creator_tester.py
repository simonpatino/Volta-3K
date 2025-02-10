from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np

# Define the parameters
n = 3  # Dimension of the state
alpha = .01 # Spread of the sigma points (small positive value)
beta = 2.  # Incorporates prior knowledge of the distribution (optimal for Gaussian)
kappa = 0.  # Secondary scaling parameter (usually 0 or 3-n)

# Create the sigma points object
sigmas = MerweScaledSigmaPoints(n, alpha, beta, kappa)

# Define the mean and covariance
x = np.array([0, 0, 0])  # Mean of the state
P = np.eye(n) * 9  # Covariance of the state (identity matrix scaled by 9)

# Compute the sigma points
sigma_points = sigmas.sigma_points(x, P)

# Print the results
print("Sigma Points:")
print(sigma_points)

print("\nWeights for the mean (Wm):")
print(sigmas.Wm)

print("\nWeights for the covariance (Wc):")
print(sigmas.Wc)