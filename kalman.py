import numpy as np

def kalman_filter(measurement, prior_mean, prior_variance, measurement_variance):
    # Measurement update
    K = prior_variance / (prior_variance + measurement_variance)
    updated_mean = prior_mean + K * (measurement - prior_mean)
    updated_variance = (1 - K) * prior_variance

    # Prediction (motion update)
    motion_variance = 0.01  # Adjust based on your system
    predicted_mean = updated_mean  # Assume no motion for simplicity
    predicted_variance = updated_variance + motion_variance

    return predicted_mean, predicted_variance

# Example usage:
initial_mean = 0  # Initial guess for the object's location (latitude or longitude)
initial_variance = 1  # Initial uncertainty
measurement_variance = 0.1  # Adjust based on GNSS noise
measurement = 2  # Received GNSS measurement (latitude or longitude)
predicted_mean, predicted_variance = kalman_filter(measurement, initial_mean, initial_variance, measurement_variance)
print(f"Predicted mean: {predicted_mean:.6f}, Predicted variance: {predicted_variance:.6f}")
