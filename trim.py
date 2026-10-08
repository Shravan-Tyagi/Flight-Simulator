# Trim Program

import numpy as np
from scipy.optimize import minimize
from aircraft_state import AircraftState
from aero_eom import derivatives


def trim_cost_function(x, state, params):
    u, w, q, delta_t, delta_e, theta = x
    
    if u <= 1.0:
        return 1e9 

    s_temp = AircraftState()
    s_temp.u = u
    s_temp.v = 0.0
    s_temp.w = w
    s_temp.p = 0.0
    s_temp.q = q
    s_temp.r = 0.0
    s_temp.phi = 0.0
    s_temp.theta = theta
    s_temp.psi = 0.0
    
    controls = [delta_t, delta_e, 0.0, 0.0]
    derivs = derivatives(s_temp, controls, params)
    
    if derivs is None or len(derivs) < 12:
        return 1e9
        
    u_dot = derivs[0]
    w_dot = derivs[2]
    q_dot = derivs[4]
    z_d_dot = derivs[11]    # level flight
    
    # Cost function with strict penalty on vertical speed and speed tracking
    cost = (u_dot**2) * 10.0 + \
           (w_dot**2) * 10.0 + \
           (q_dot**2) * 100.0 + \
           (z_d_dot**2) * 10000.0 + \
           ((s_temp.u - 50.0)**2)
           
    return cost


def run_trim(x0, state, params):
    result = minimize(trim_cost_function, x0, args=(state, params), method='Nelder-Mead', options={'maxiter': 1000, 'xatol': 1e-6, 'fatol': 1e-6})

    if result.success or result.nit > 0:
        u_trim, w_trim, q_trim, dt_trim, de_trim, theta_trim = result.x
        
        # Apply trim values to main simulation state
        state.u = u_trim
        state.w = w_trim
        state.q = q_trim
        state.theta = theta_trim
        state.v = 0.0
        state.p = 0.0
        state.r = 0.0
        state.phi = 0.0
        state.psi = 0.0
        
        trimmed_controls = [dt_trim, de_trim, 0.0, 0.0]
        print(f"\n--- TRIM FOUND ---")
        print(f"Throttle: {dt_trim:.3f}")
        print(f"Elevator: {de_trim:.4f} rad ({np.degrees(de_trim):.2f}°)")
        print(f"Pitch (theta): {theta_trim:.4f} rad ({np.degrees(theta_trim):.2f}°)")
        print(f"Alpha: {np.arctan2(w_trim, u_trim):.4f} rad ({np.degrees(np.arctan2(w_trim, u_trim)):.2f}°)")
        print(f"Final Cost: {result.fun:.6f}")
        
        return trimmed_controls
    
    else:
        print(f"Optimization failed: {result.message}")
        trimmed_controls = [0.392, -0.094, 0.0, 0.0]
        return trimmed_controls
    






