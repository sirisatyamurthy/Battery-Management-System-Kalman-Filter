import numpy as np

class KalmanFilter:

    def __init__(self):

        # Initial SOC estimate
        self.soc = 1.0

        # Initial error covariance
        self.P = 1.0

        # Process noise
        self.Q = 0.0001

        # Measurement noise
        self.R = 0.01


    def predict(self, current, dt, capacity):

        # Predict next SOC
        self.soc -= (current * dt) / (3600 * capacity)

        # Update covariance
        self.P += self.Q


    def update(self, measured_voltage):

        # Predicted voltage from SOC
        predicted_voltage = (
            3.0
            + 0.7 * self.soc
            + 0.1 * np.sin(5 * self.soc)
            + 0.4 * self.soc**2
        )
        # Kalman Gain
        K = self.P / (self.P + self.R)

        # Measurement error
        error = measured_voltage - predicted_voltage

        # Correct SOC estimate
        self.soc += K * error

        # Update covariance
        self.P *= (1 - K)

        # Clamp SOC
        self.soc = max(0, min(1, self.soc))