
import numpy as np

class Formula_E():
    def __init__(self):
        ### Vehicle Parameters ###
        # Geometry
        self.L = 2.97 # m
        self.length = 5.02 # m
        self.width = 1.70 # m 
        self.A = 1.1 # m^2 (frontal Area)
        self.r_front = 0.65 # m (front tire diameter)
        self.r_rear = 0.69 # m (rear tire diameter)
        self.phi_max = 25 * np.pi/180 # rad

        # Inertia
        self.m = 840 # kg (including driver)
        self.h = 0.3 # m  (C.o.G. height)

        # Powertrain
        self.P = 300 # kW
        self.F_max = mu * f_n(v) * g
        self.v_cross = P/F_max

        # Aerodynamics
        self.C_D = 0.95
        self.C_L = 0.95
        self.rho = 1.225 # kg/m^3 (sea level)
        self.g = 9.81 # m/s^2
        self.C_rr = 0.015 # Rolling Resistance (optional)

        # Tires
        self.mu = 1.5 # Estimate. 1.8 for full slicks, 1.0 for road tires


    def f_drag(self, v): 1050 * (v/50)**2 # N

    def f_down(self, v): 1030 * (v/50)**2 # N

    def f_max_grip(self, v): self.mu * self.f_n(v)

    def f_n(self, v): self.m*self.g + self.f_down(v)

    def f_motor(self, v): 
        min(T_max/r_wheel, P/v)