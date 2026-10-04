# Aircraft Initial States

class AircraftState:
    def __init__(self):
        # Translational velocities (m/s)
        self.u = 50
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
        self.x_n = 0
        self.y_e = 0
        self.z_d = -100

