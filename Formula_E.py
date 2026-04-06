
import numpy as np

class Formula_E():
    def __init__(self):
        ### Vehicle Parameters ###
        # Geometry
        self.L = 2.97 # m
        self.length = 5.02 # m
        self.width = 1.70 # m 
        self.A = 1.1 # m^2 (frontal Area)
        self.phi_max = 25 * np.pi/180 # rad

        # Tires
        self.mu = 1.5 # Estimate. 1.8 for full slicks, 1.0 for road tires
        self.r_front = 0.65/2 # m (front tire diameter)
        self.r = 0.69/2 # m (rear tire diameter)

        # Inertia
        self.m = 840 # kg (including driver)
        self.h = 0.3 # m  (C.o.G. height)

        # Powertrain
        self.P_max = 300 * 1000 # W
        self.F_max = self.mu * self.f_n(0) * self.g
        self.brake_max = -self.F_max # Assume the brakes are capable of locking tires at any point
        self.v_cross = self.P/self.F_max

        # Aerodynamics
        self.C_D = 0.95
        self.C_L = 0.95
        self.rho = 1.225 # kg/m^3 (sea level)
        self.g = 9.81 # m/s^2
        self.C_rr = 0.015 # Rolling Resistance (optional)

        # Initial Conditions:
        self.pos = (0, 0)
        self.phi = 0
        self.theta = 0

    def rand_control(self):
        c = np.random.random_sample((2, 1)) # FIXME: SCALE THIS

    def theta_dot(self, v, phi):
        theta_dot = v/self.L*np.atan(phi)

    def update_pos(self, v, phi, theta):
        self.pos[0] += v*np.cos(theta)
        self.pos[1] += v*np.sin(theta)
        theta += self.theta_dot(v, phi)

        r_mat = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta),  np.cos(theta)]])

        car =  [
                r_mat @ np.array([0,  self.width]) + self.pos, # Rear left
                r_mat @ np.array([0, -self.width]) + self.pos, # Rear right
                r_mat @ np.array([2*self.length, -self.width]) + self.pos, # Front left
                r_mat @ np.array([2*self.length,  self.width]) + self.pos, # Front right
            ]


    def f_drag(self, v): 1050 * (v/50)**2 # N (From Linkedin AirShaper CFD)

    def f_down(self, v): 1030 * (v/50)**2 # N (From Linkedin AirShaper CFD)

    def f_max_grip(self, v): self.mu * self.f_n(v)

    def f_n(self, v): self.m*self.g + self.f_down(v)

    def f_motor(self, v): 
        min(self.f_max_grip(v), self.P/v)

    def max_turn_allowed(self, v):

        min()

    def friction_circle(self, v, acc, phi):

        fx = self.m * acc
        fy = r * theta_dot **2 


        return (fx**2 + fy**2) / (self.mu * self.f_n(v)) ** 2 <= 1
        
