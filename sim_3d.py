# Flight Simulator 3D
# Author: Shravan

import numpy as np
from ursina import *
from controller import PIDController
from aircraft import AircraftParameters
from aircraft_state import AircraftState
from aero_eom import derivatives
from solver import rk4_step
import trim


# Initialize Ursina App
app = Ursina()

# Window setup
window.title = "3D 6-DOF Flight Simulator"
window.borderless = False
window.color = color.light_gray
window.fps_counter.enabled = True

# Initialize Aircraft State & Parameters
state = AircraftState()
params = AircraftParameters()

# Trim
x0 = [50, 50 * 0.0432, 0, 0.392, -0.094, 0.0432]
trimmed_controls = list(trim.run_trim(x0, state, params))
current_controls = list(trimmed_controls)


# 3D Environment Setup
ground = Entity(model='plane', scale=(20000, 1, 20000), color=color.green.tint(-0.3), texture='white_cube', texture_scale=(200, 200))
Sky()


# Aircraft Body
plane_body = Entity(model='cube', scale=(2, 0.4, 6), color=color.blue)
wing = Entity(model='cube', scale=(12, 0.1, 1.8), parent=plane_body, color=color.white)
tail = Entity(model='cube', scale=(2, 1.5, 0.2), position=(0, 0.5, -2.5), parent=plane_body, color=color.white)


hud_text = Text(text='', position=(-0.85, 0.45), scale=1.1, color=color.black)
camera.fov = 75


# Real-Time Simulation Update Loop
def update():
    global state, current_controls
    
    # Get delta time from Ursina frame clock (capped to avoid instability)
    dt = min(time.dt, 0.05)
    if dt == 0:
        return

    # Keyboard Controls
    if held_keys['w']:
        current_controls[0] = min(1.0, current_controls[0] + 0.1 * dt)
    if held_keys['s']:
        current_controls[0] = max(0.0, current_controls[0] - 0.1 * dt)

    elevator_input = current_controls[1]
    if held_keys['up arrow']:
        elevator_input -= 0.05 * dt
    elif held_keys['down arrow']:
        elevator_input += 0.05 * dt
    else:
        # Return slightly toward trim when released
        elevator_input += (trimmed_controls[1] - elevator_input) * 2.0 * dt

    aileron_input = 0.0
    if held_keys['left arrow']:
        aileron_input -= 0.1
    if held_keys['right arrow']:
        aileron_input += 0.1

    rudder_input = 0.0
    if held_keys['a']:
        rudder_input += 0.1
    if held_keys['d']:
        rudder_input -= 0.1

    # Clip control surfaces
    current_controls[1] = np.clip(elevator_input, -0.2, 0.2)
    current_controls[2] = np.clip(aileron_input, -0.15, 0.15)
    current_controls[3] = np.clip(rudder_input, -0.15, 0.15)

    # Solver
    state = rk4_step(state, current_controls, params, dt)

    # Coordinate Mapping: NED to Ursina 3D Space
    # NED: x_n (North -> Z), y_e (East -> X), z_d (Down -> -Y Altitude)
    plane_body.position = Vec3(state.y_e, -state.z_d, state.x_n)

    roll_deg = np.degrees(state.phi)
    pitch_deg = np.degrees(state.theta)
    yaw_deg = np.degrees(state.psi)

    # Ursina rotation order mapping: (pitch, yaw, roll)
    plane_body.rotation = Vec3(-pitch_deg, yaw_deg, roll_deg)

    # Chase Camera
    camera.position = plane_body.position - (plane_body.forward * 18) + Vec3(0, 4, 0)
    camera.look_at(plane_body.position + Vec3(0, 1, 0))

    # Update HUD Telemetry
    hud_text.text = (
        f"=== FLIGHT TELEMETRY ===\n"
        f"Altitude: {-state.z_d:.1f} m\n"
        f"Airspeed (u): {state.u:.1f} m/s\n"
        f"Throttle: {current_controls[0]*100:.1f}%\n"
        f"Pitch: {pitch_deg:.1f} deg | Roll: {roll_deg:.1f} deg | Yaw: {yaw_deg:.1f} deg\n\n"
        f"=== CONTROLS ===\n"
        f"Throttle: W / S\n"
        f"Pitch (Elevator): Up / Down Arrows\n"
        f"Roll (Aileron): Left / Right Arrows\n"
        f"Yaw (Rudder): A / D"
    )

app.run()
