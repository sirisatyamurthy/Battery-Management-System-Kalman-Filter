from battery_model import Battery

battery = Battery()

print("Initial SOC:", battery.soc)
print("Initial Voltage:", battery.get_voltage())

battery.update(current=1.0, dt=10)

print("Updated SOC:", battery.soc)
print("Updated Voltage:", battery.get_voltage())