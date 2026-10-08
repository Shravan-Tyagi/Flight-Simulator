# Flight Simulator
# 4/10/2026
# Author: Shravan

import numpy as np
import matplotlib.pyplot as plt
from aircraft import AircraftParameters
from aircraft_state import AircraftState
from controller import PIDController
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
target_history, elevator_history = [], []

# Trim
x0 = [50, 50 * 0.0432, 0, 0.392, -0.094, 0.0432]
trimmed_controls = trim.run_trim(x0, state, params)


# Initialize Altitude PID Controller
alt_pid = PIDController(kp=0.2, ki=0.05, kd=0.2, output_limits=(-0.15, 0.15))
target_altitude = 1000



for i in range(steps):
    t = i * dt
    
    controls = trimmed_controls   # [Throttle, Elevator, Aileron, Rudder]

    # Controls Test:
    #if 40 <= t <= 45:
    #    controls = [0.325, -0.1, 0, 0]
    #else:
    #    controls = trimmed_controls 


    # PID Test:
    if t >= 20:
        target_altitude = 1200

    current_altitude = -state.z_d
    
    elevator_adjustment = -alt_pid.compute(target_altitude, current_altitude, dt)
    current_elevator = np.clip(trimmed_controls[1] + elevator_adjustment, -0.2, 0.2)

    print(t, trimmed_controls[1], elevator_adjustment)
    controls = [trimmed_controls[0], current_elevator, trimmed_controls[2], trimmed_controls[3]]
    #controls = trimmed_controls


    # Solver Run
    state = rk4_step(state, controls, params, dt)
    
    time_history.append(t)
    alt_history.append(-state.z_d)
    pitch_history.append(np.degrees(state.theta))
    alpha_history.append(np.degrees(np.arctan2(state.w, state.u)))
    u_history.append(state.u)
    target_history.append(target_altitude)
    elevator_history.append(np.degrees(current_elevator))



# Plot
plt.figure(figsize=(12, 8))
plt.subplot(3, 2, 1)
plt.plot(time_history, alt_history, color='b')
plt.title("6-DOF Flight Dynamics Simulation Response")
plt.ylabel("Altitude MSL (m)")
plt.grid(True)

plt.subplot(3, 2, 2)
plt.plot(time_history, pitch_history, color='r')
plt.ylabel("Theta (deg)")
plt.grid(True)

plt.subplot(3, 2, 3)
plt.plot(time_history, alpha_history, color='y')
plt.ylabel("Alpha (deg)")
plt.grid(True)

plt.subplot(3, 2, 4)
plt.plot(time_history, u_history, color='g')
plt.xlabel("Time (s)")
plt.ylabel("Forward Speed u (m/s)")
plt.grid(True)

plt.subplot(3, 2, 5)
plt.plot(time_history, alt_history, label="Actual Altitude [m]", color='b')
plt.plot(time_history, target_history, label="Target Altitude [m]", color='r', linestyle='--')
plt.ylabel("Altitude [m]")
plt.legend()
plt.grid(True)

plt.subplot(3, 2, 6)
plt.plot(time_history, elevator_history, color='g')
plt.xlabel("Time [s]")
plt.ylabel("Elevator Deflection [deg]")
plt.grid(True)

plt.tight_layout()
plt.show()









