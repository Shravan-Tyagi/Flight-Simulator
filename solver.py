# Solver

import numpy as np
from aero_eom import derivatives
from aircraft_state import AircraftState


def rk4_step(state, controls, params, dt):
    """Runge-Kutta 4th order numerical integration scheme."""

    def state_to_array(s):
        return np.array([s.u, s.v, s.w, s.p, s.q, s.r, s.phi, s.theta, s.psi, s.x_n, s.y_e, s.z_d])
    
    def array_to_state(arr, s):
        s.u, s.v, s.w, s.p, s.q, s.r, s.phi, s.theta, s.psi, s.x_n, s.y_e, s.z_d = arr
        return s

    y0 = state_to_array(state)
    
    k1 = derivatives(state, controls, params)
    k2 = derivatives(array_to_state(y0 + 0.5 * dt * k1, AircraftState()), controls, params)
    k3 = derivatives(array_to_state(y0 + 0.5 * dt * k2, AircraftState()), controls, params)
    k4 = derivatives(array_to_state(y0 + dt * k3, AircraftState()), controls, params)
    
    y_next = y0 + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    updated_state = array_to_state(y_next, state)
    
    return updated_state

