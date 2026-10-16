# Aircraft Initial States

class AircraftState:
    def __init__(self, u = 50, n = 0, e = 0, d = 0):
        # Translational velocities (m/s)
        self.u = u
        self.v = 0
        self.w = 0
        
        # Angular rates (rad/s)
        self.p = 0
        self.q = 0
        self.r = 0
        
        # Euler angles (rad)
        self.phi = 0
        self.theta = 0 
        self.psi = 0
        
        # Position (m)
        self.x_n = n
        self.y_e = e
        self.z_d = d

