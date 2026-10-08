# Flight Simulator
# 4/10/2026
# Author: Shravan

import numpy as np
import matplotlib.pyplot as plt
from aircraft import AircraftParameters
from aircraft_state import AircraftState
from solver import rk4_step
import trim


# Simulation Setup
dt = 0.01  # seconds
total_time = 100
steps = int(total_time / dt)

state = AircraftState()
params = AircraftParameters()

# History tracking for plotting
time_history = []
alt_history = []
pitch_history = []
alpha_history = []
u_history = []

# Trim
x0 = [50, 50 * 0.0432, 0, 0.392, -0.094, 0.0432]
trimmed_controls = trim.run_trim(x0, state, params)


for i in range(steps):
    t = i * dt
    
    controls = trimmed_controls   # [Throttle, Elevator, Aileron, Rudder]

    # Controls Test:
    if 40 <= t <= 45:
        controls = [0.325, -0.1, 0, 0]
    else:
        controls = trimmed_controls 


    # Solver Run
    state = rk4_step(state, controls, params, dt)
    
    time_history.append(t)
    alt_history.append(-state.z_d)
    pitch_history.append(np.degrees(state.theta))
    alpha_history.append(np.degrees(np.arctan2(state.w, state.u)))
    u_history.append(state.u)



# Plot
plt.figure(figsize=(12, 8))
plt.subplot(4, 1, 1)
plt.plot(time_history, alt_history, color='b')
plt.title("6-DOF Flight Dynamics Simulation Response")
plt.ylabel("Altitude (m)")
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(time_history, pitch_history, color='r')
plt.ylabel("Theta (deg)")
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(time_history, alpha_history, color='y')
plt.ylabel("Alpha (deg)")
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(time_history, u_history, color='g')
plt.xlabel("Time (s)")
plt.ylabel("Forward Speed u (m/s)")
plt.grid(True)


plt.tight_layout()
plt.show()









