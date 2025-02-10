import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints

# Define the state dimension
n = 4  # Change this to test for different dimensions

# Create the sigma points generator
sp = MerweScaledSigmaPoints(n=n, alpha=0.1, beta=2.0, kappa=0)

# Define mean and covariance
x = np.zeros(n)  # Mean state vector
P = np.eye(n)    # Covariance matrix

# Generate sigma points
sigma_points = sp.sigma_points(x, P)

# Print the shape of the sigma points matrix
print("Sigma points matrix shape:", sigma_points.shape)

# Assert to prove the shape is (2n + 1, n)
assert sigma_points.shape == (2 * n + 1, n), "Matrix size is incorrect!"
