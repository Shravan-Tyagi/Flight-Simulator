# Aircraft Controller

import numpy as np


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1, 1)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_limit, self.max_limit = output_limits
        
        self.integral = 0
        self.previous_error = 0

    def compute(self, target, current, dt):
        error = target - current
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        self.previous_error = error
        
        # Sum components
        output = p_term + i_term + d_term
        
        # Clamping output
        return np.clip(output, self.min_limit, self.max_limit)
