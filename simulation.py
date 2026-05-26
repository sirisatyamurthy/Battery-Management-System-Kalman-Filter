import numpy as np
import matplotlib.pyplot as plt

from sklearn.metrics import mean_squared_error
from battery_model import Battery
from kalman_filter import KalmanFilter


# Create battery object
battery = Battery()

# Create Kalman Filter object
kf = KalmanFilter()


# Simulation settings
dt = 1
time_steps = 2000
coulomb_soc = 1.0


# Lists for plotting
true_soc = []
estimated_soc = []
measured_voltages = []
currents = []
errors = []
cc_soc = []
tte_minutes = []

for t in range(time_steps):

    # Simulated varying current draw
    current = 0.5 + 0.5 * np.sin(t / 100)

    # Update true battery
    battery.update(current, dt)

    # Coulomb Counting Update

    # Simulated current sensor bias

    biased_current = current + 0.02

    coulomb_soc -= (biased_current * dt) / (3600 * battery.capacity)
    coulomb_soc = max(0, min(1, coulomb_soc))

    # True battery voltage
    true_voltage = battery.get_voltage()

    # Add sensor noise
    noise = np.random.normal(0, 0.02)

    measured_voltage = true_voltage + noise

    # Kalman prediction step
    kf.predict(current, dt, battery.capacity)

    # Kalman correction step
    kf.update(measured_voltage)

    # Store values
    true_soc.append(battery.soc * 100)

    estimated_soc.append(kf.soc * 100)

    cc_soc.append(coulomb_soc * 100)

    measured_voltages.append(measured_voltage)

    currents.append(current)

    errors.append((battery.soc - kf.soc) * 100)

    # Time-To-Empty calculation

    if current > 0.01:

        remaining_capacity = kf.soc * battery.capacity

        tte = (remaining_capacity / current) * 60

    else:

        tte = 0

    tte_minutes.append(tte)

fig, axs = plt.subplots(5, 1, figsize=(12, 20))

# ---------------------------------------------------
# SOC Plot
# ---------------------------------------------------

axs[0].plot(true_soc, label="True SOC")

axs[0].plot(estimated_soc, label="Estimated SOC")

axs[0].plot(true_soc, label="True SOC")

axs[0].plot(estimated_soc, label="Kalman SOC")

axs[0].plot(cc_soc, label="Coulomb Counting SOC")

axs[0].set_title("SOC Estimation using Kalman Filter")

axs[0].set_xlabel("Time Step")

axs[0].set_ylabel("SOC (%)")

axs[0].legend()

axs[0].grid()


# ---------------------------------------------------
# Current Plot
# ---------------------------------------------------

axs[1].plot(currents)

axs[1].set_title("Battery Current Profile")

axs[1].set_xlabel("Time Step")

axs[1].set_ylabel("Current (A)")

axs[1].grid()


# ---------------------------------------------------
# Voltage Plot
# ---------------------------------------------------

axs[2].plot(measured_voltages)

axs[2].set_title("Measured Battery Voltage")

axs[2].set_xlabel("Time Step")

axs[2].set_ylabel("Voltage (V)")

axs[2].grid()


# ---------------------------------------------------
# Error Plot
# ---------------------------------------------------

axs[3].plot(errors)

axs[3].set_title("SOC Estimation Error")

axs[3].set_xlabel("Time Step")

axs[3].set_ylabel("Error (%)")

axs[3].grid()

# ---------------------------------------------------
# Time-To-Empty Plot
# ---------------------------------------------------

axs[4].plot(tte_minutes)

axs[4].set_title("Predicted Time-To-Empty")

axs[4].set_xlabel("Time Step")

axs[4].set_ylabel("Minutes Remaining")

axs[4].grid()

# Layout adjustment
plt.tight_layout()


# Save figure
plt.savefig("plots/dashboard.png", dpi=300)


# RMSE Calculation
kalman_rmse = np.sqrt(mean_squared_error(true_soc, estimated_soc))

cc_rmse = np.sqrt(mean_squared_error(true_soc, cc_soc))

print(f"Kalman Filter RMSE: {kalman_rmse:.4f} %")

print(f"Coulomb Counting RMSE: {cc_rmse:.4f} %")


# Show plots
plt.show()