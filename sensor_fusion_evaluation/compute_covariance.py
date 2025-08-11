import pandas as pd
import numpy as np

# Load the CSV file
df = pd.read_csv("imu_data.csv")

# Select numeric columns (ignore time for covariance calculation)
data = df[['qx', 'qy', 'qz', 'qw',
           'ax', 'ay', 'az',
           'gx', 'gy', 'gz']]

# Convert dataframe to numpy
data_np = data.to_numpy()

# Compute covariance matrix (variables in columns, observations in rows)
cov_matrix = np.cov(data_np, rowvar=False)

# Print result
print("Covariance Matrix:")
print(cov_matrix)

# Also save to CSV if needed
np.savetxt("imu_covariance_matrix.csv", cov_matrix, delimiter=",")
