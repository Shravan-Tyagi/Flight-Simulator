# Aerodynamics, Propulsion and EOM

import numpy as np


def derivatives(state, controls, params):
    # Newton-Euler rigid body EOM.
    
    u, v, w = state.u, state.v, state.w
    p, q, r = state.p, state.q, state.r
    phi, theta, psi = state.phi, state.theta, state.psi
    
    delta_t, delta_e, delta_a, delta_r = controls

    altitude = max(0.0, -state.z_d)
    rho = 1.22
    
    # Kinematics
    V_total = np.sqrt(u**2 + v**2 + w**2)
    alpha = np.arctan2(w, u) if u != 0 else 0.0
    beta = np.arcsin(np.clip(v / V_total, -1.0, 1.0)) if V_total != 0 else 0.0
    
    qbar = 0.5 * rho * V_total**2
    
    # Force Coefficients
    CL = 0.2 + 5.5 * alpha + 0.4 * delta_e
    CD = 0.04 + 0.05 * (CL**2)
    CY = -0.4 * beta + 0.3 * delta_r
    
    # Moment Coefficients
    Cl = -0.1 * beta - 0.15 * p * params.b / (2 * V_total) + 0.1 * delta_a
    Cm = -0.05 - 0.8 * alpha - 1.2 * q * params.c / (2 * V_total) - 0.8 * delta_e
    Cn = 0.05 * beta - 0.2 * r * params.b / (2 * V_total) - 0.2 * delta_r
    
    # Forces and Moments
    Lift = qbar * params.S * CL
    Drag = qbar * params.S * CD
    Side = qbar * params.S * CY
    
    L = qbar * params.S * params.b * Cl
    M = qbar * params.S * params.c * Cm
    N = qbar * params.S * params.b * Cn
    
    # Propulsion
    Thrust = delta_t * 3000
    
    # Wind to Body Frame
    Fx_aero = -Drag * np.cos(alpha) + Lift * np.sin(alpha)
    Fy_aero = Side
    Fz_aero = -Drag * np.sin(alpha) - Lift * np.cos(alpha)
    
    # Total Body Forces
    Fx = Fx_aero + Thrust
    Fy = Fy_aero
    Fz = Fz_aero + params.mass * params.g * np.sin(theta)
    
    # EOM
    u_dot = (Fx / params.mass) - (q * w - r * v)
    v_dot = (Fy / params.mass) - (r * u - p * w)
    w_dot = (Fz / params.mass) - (p * v - q * u)

    p_dot = L/params.Ixx
    q_dot = M/params.Iyy
    r_dot = N/params.Izz
    
    phi_dot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
    theta_dot = q * np.cos(phi) - r * np.sin(phi)
    psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)
    
    # Body Velocities to Inertial
    c_th, s_th = np.cos(theta), np.sin(theta)
    c_ph, s_ph = np.cos(phi), np.sin(phi)
    c_ps, s_ps = np.cos(psi), np.sin(psi)
    
    x_n_dot = u * c_th * c_ps + v * (s_ph * s_th * c_ps - c_ph * s_ps) + w * (c_ph * s_th * c_ps + s_ph * s_ps)
    y_e_dot = u * c_th * s_ps + v * (s_ph * s_th * s_ps + c_ph * c_ps) + w * (c_ph * s_th * s_ps - s_ph * c_ps)
    z_d_dot = -u * s_th + v * s_ph * c_th + w * c_ph * c_th
    
    return np.array([u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot, x_n_dot, y_e_dot, z_d_dot])




