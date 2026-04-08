
import numpy as np
rand = np.random.default_rng()


class Formula_E():
    class car_state():
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

        
        def pos(self):
            return np.array([self.x, self.y])

    def __init__(self, x=0, y=0, theta=0, scale=1):
        ### Vehicle Parameters ###
        # Geometry
        self.L = 2.97 # m
        self.length = 5.02 *scale # m (scaled for drawing)
        self.width = 1.70 * scale # m (scaled for drawing)
        self.A = 1.1 # m^2 (frontal Area)
        self.phi_max = 25 * np.pi/180 # rad

        # Tires
        self.mu = 1.5 # Estimate. 1.8 for full slicks, 1.0 for road tires
        self.r_front = 0.65/2 # m (front tire diameter)
        self.r = 0.69/2 # m (rear tire diameter)

        # Inertia
        self.m = 840 # kg (including driver)
        self.h = 0.3 # m  (C.o.G. height)

        # Aerodynamics
        self.C_D = 0.95
        self.C_L = 0.95
        self.rho = 1.225 # kg/m^3 (sea level)
        self.g = 9.81 # m/s^2
        self.C_rr = 0.015 # Rolling Resistance (optional)

        # Powertrain
        self.P_max = 300 * 1000 # W
        self.f_max = self.mu * self.f_n(0) * self.g
        self.brake_max = -self.f_max # Assume the brakes are capable of locking tires at any point
        self.v_cross = self.P_max/self.f_max
        self.v_max = 320 / 3.6 # kph -> m/s
        

        # Initial Conditions:
        self.state = self.car_state(x, y, theta)
        self.phi = 0
        self.v = 0

    def rand_control(self):
        phi = rand.uniform(-self.phi_max, self.phi_max)
        acc = rand.uniform(-self.F_max, self.P_max) 
        # Min braking power is always traction limited
        # Full throttle is the max sample, it may be reduced later.
        return (acc, phi)

    def theta_dot(self, v, phi): return v/self.L*np.atan(phi)

    def update_pos(self): # FIXME: Close but not fully right.
        theta = self.state.theta
        self.state.x += self.v*np.cos(theta)
        self.state.y += self.v*np.sin(theta)
        self.state.theta += self.theta_dot(self.v, self.phi)
        theta = self.state.theta

        r_mat = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta),  np.cos(theta)]])

        car_box =  [
                    r_mat @ np.array([0,  self.width]) + self.state.pos(), # Rear left
                    r_mat @ np.array([0, -self.width]) + self.state.pos(), # Rear right
                    r_mat @ np.array([2*self.length, -self.width]) + self.state.pos(), # Front left
                    r_mat @ np.array([2*self.length,  self.width]) + self.state.pos(), # Front right
                   ]
        
        return car_box


    def f_drag(self, v): return 1050 * (v/50)**2 # N (From Linkedin AirShaper CFD)

    def f_down(self, v): return 1030 * (v/50)**2 # N (From Linkedin AirShaper CFD)

    def f_max_grip(self, v): return self.mu * self.f_n(v)

    def f_n(self, v): return self.m * self.g + self.f_down(v)

    def f_motor(self, v): return min(self.f_max_grip(v), self.P/v)

    def scale_control(self, v, acc, phi):
        f_gas, f_turn = self.calc_forces(v, acc, phi)
        f_mag = np.sqrt(f_gas**2, f_turn**2)
        self.f_max = self.f_max_grip(v)

        if f_mag < self.max_f:
            return (acc, phi)
        else:
            return (acc/f_mag*self.f_max, np.atan(np.tan(phi)/f_mag*self.f_max)) # FIXME: Check this.

    def calc_forces(self, v, acc, phi):
        f_gas = self.m * acc

        # atan(phi) = L/R
        R = self.L/np.tan(phi)
        f_turn = self.m * v**2 / R

        return (f_gas, f_turn)
