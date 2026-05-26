import numpy as np

class Battery:

    def __init__(self, capacity_ah=2.0):

        # Battery capacity in Ah
        self.capacity = capacity_ah

        # Initial SOC = 100%
        self.soc = 1.0


    def update(self, current, dt):

        # Coulomb counting equation
        self.soc -= (current * dt) / (3600 * self.capacity)

        # Limit SOC between 0 and 1
        self.soc = max(0, min(1, self.soc))


    def get_voltage(self):

        # Nonlinear Li-ion OCV model

        soc = self.soc

        voltage = (
            3.0
            + 0.7 * soc
            + 0.1 * np.sin(5 * soc)
            + 0.4 * soc**2
        )
        return voltage