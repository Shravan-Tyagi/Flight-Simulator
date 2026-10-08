# Atmosphere Module


def get_indian_atmosphere(altitude):
    """
    Computes air density using an Indian Tropical/Sub-tropical 
    reference model (Base: T0 = 303.15 K, P0 = 100500 Pa, rho0 = 1.155 kg/m^3).
    Altitude is given in meters above MSL.
    """
    h = max(0.0, altitude)
    
    T_0 = 303.15        # Surface Temp [K] (30 C)
    p_0 = 100500.0      # Surface Pressure [Pa]
    g = 9.81            # Gravity
    R = 287.05          # Specific gas constant for dry air
    lapse_rate = 0.0065 # Standard temperature lapse rate [K/m]
    
    T = T_0 - (lapse_rate * h)
    if T < 216.65:
        T = 216.65
        
    p = p_0 * (T / T_0) ** (g / (lapse_rate * R))
    rho = p / (R * T)
    return rho


if __name__ == "__main__":
    print(get_indian_atmosphere(20000))
