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
#plane_body = Entity(model='cube', scale=(2, 0.4, 6), color=color.blue)
#wing = Entity(model='cube', scale=(12, 0.1, 1.8), parent=plane_body, color=color.white)
#tail = Entity(model='cube', scale=(2, 1.5, 0.2), position=(0, 0.5, -2.5), parent=plane_body, color=color.white)


# Terrain Cubes
np.random.seed(42)
for _ in range(100):
    rx = np.random.uniform(-10000, 10000)
    rz = np.random.uniform(-10000, 10000)
    Entity(model='cube', scale=(np.random.uniform(10, 30), np.random.uniform(20, 80), np.random.uniform(10, 30)),
           position=(rx, 0, rz), color=color.rgb(90, 90, 90))


# Better Aircraft Body
plane_body = Entity()

# Main structure
fuselage = Entity(model='cube', scale=(1.6, 1.2, 6.5), color=color.rgb(142/255, 142/255, 142/255), parent=plane_body)
nose = Entity(model='sphere', scale=(1.5, 1.1, 2.0), position=(0, 0, 3.0), color=color.rgb(20/255, 80/255, 180/255), parent=plane_body)
cockpit = Entity(model='cube', scale=(1.3, 0.7, 1.8), position=(0, 0.6, 0.6), color=color.rgb(180/255, 220/255, 255/255), parent=plane_body)

# Wings
left_wing = Entity(model='cube', scale=(6.0, 0.12, 1.6), position=(-3.4, 0.1, 0.2), color=color.rgb(142/255, 142/255, 142/255), parent=plane_body)
right_wing = Entity(model='cube', scale=(6.0, 0.12, 1.6), position=(3.4, 0.1, 0.2), color=color.rgb(142/255, 142/255, 142/255), parent=plane_body)
left_aileron = Entity(model='cube', scale=(2.0, 0.1, 0.4), position=(-4.8, 0.1, -0.6), color=color.rgb(220/255, 50/255, 50/255), parent=plane_body)
right_aileron = Entity(model='cube', scale=(2.0, 0.1, 0.4), position=(4.8, 0.1, -0.6), color=color.rgb(220/255, 50/255, 50/255), parent=plane_body)

# Tail
horiz_stabilizer = Entity(model='cube', scale=(3.5, 0.08, 1.0), position=(0, 0.3, -3.0), color=color.rgb(142/255, 142/255, 142/255), parent=plane_body)
elevator = Entity(model='cube', scale=(3.2, 0.06, 0.4), position=(0, 0.3, -3.6), color=color.rgb(220/255, 50/255, 50/255), parent=plane_body)
vert_fin = Entity(model='cube', scale=(0.12, 1.6, 1.2), position=(0, 1.0, -2.8), color=color.rgb(30/255, 100/255, 220/255), parent=plane_body)
rudder = Entity(model='cube', scale=(0.1, 1.4, 0.5), position=(0, 0.9, -3.5), color=color.rgb(220/255, 50/255, 50/255), parent=plane_body)



hud_text = Text(text='', position=(-0.85, 0.45), scale=1.1, color=color.black)

# Camera configuration & Mouse Orbit variables
camera.fov = 75
cam_distance = 18
cam_height = 4
orbit_x = 0
orbit_y = 0
is_orbiting = False


def input(key):
    global is_orbiting, cam_distance, orbit_x, orbit_y
    if key == 'left mouse down':
        is_orbiting = True
    elif key == 'left mouse up':
        is_orbiting = False
    elif key == 'r':
        # Reset POV and zoom
        orbit_x = 0
        orbit_y = 0
        cam_distance = 18
    elif key == 'scroll up':
        cam_distance = max(6, cam_distance - 2)
    elif key == 'scroll down':
        cam_distance = min(50, cam_distance + 2)



# Real-Time Simulation Update Loop
def update():
    global state, current_controls, orbit_x, orbit_y, cam_distance
    
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

    # Animate Control Surfaces
    elevator.rotation_x = np.degrees(current_controls[1]) * 40.0
    left_aileron.rotation_x = -np.degrees(current_controls[2]) * 40.0
    right_aileron.rotation_x = np.degrees(current_controls[2]) * 40.0
    rudder.rotation_y = -np.degrees(current_controls[3]) * 40.0


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

    # Mouse Orbit & Chase Camera
    if is_orbiting:
        orbit_y += mouse.velocity[0] * 150
        orbit_x -= mouse.velocity[1] * 150

    # Base chase camera position anchored to plane
    base_cam_pos = plane_body.position - (plane_body.forward * cam_distance) + Vec3(0, cam_height, 0)
    
    # Apply mouse orbit rotation around the aircraft
    if is_orbiting or orbit_x != 0 or orbit_y != 0:
        offset = base_cam_pos - plane_body.position
        # Apply simple orbit offsets
        camera.position = plane_body.position + offset + Vec3(orbit_y * 0.1, orbit_x * 0.1, 0)
    else:
        camera.position = base_cam_pos

    camera.look_at(plane_body.position + Vec3(0, 1, 0))


    # Update HUD Telemetry
    hud_text.text = (
        f"=== FLIGHT TELEMETRY ===\n"
        f"Altitude: {-state.z_d:.1f} m\n"
        f"Airspeed (u): {state.u:.1f} m/s\n"
        f"Throttle: {current_controls[0]*100:.1f}%\n"
        f"Pitch: {pitch_deg:.1f} deg | Roll: {roll_deg:.1f} deg | Yaw: {yaw_deg:.1f} deg\n"
        f"Controls: [{current_controls[0]*100:.1f}, {current_controls[1]*57.29:.1f},{current_controls[2]*57.29:.1f},{current_controls[3]*57.29:.1f}],\n\n"
        f"=== CONTROLS ===\n"
        f"Throttle: W / S\n"
        f"Pitch (Elevator): Up / Down Arrows\n"
        f"Roll (Aileron): Left / Right Arrows\n"
        f"Yaw (Rudder): A / D\n"
        f"Orbit POV: Left Click + Drag | Zoom: Scroll Wheel | Reset POV: R"
    )

app.run()
