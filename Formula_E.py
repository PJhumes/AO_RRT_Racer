import shapely.geometry as geom
import numpy as np
rand = np.random.default_rng()

class Car_State():
        def __init__(self, x, y, theta, v=0, proj=0):
            self.x = x
            self.y = y
            self.theta = theta
            self.proj = proj
            self.v = v

        def pos(self):
            return np.array([self.x, self.y])
        
        def copy(self):
            return Car_State(self.x, self.y, self.theta, self.v, self.proj)
        
        def __str__(self):
            return str((self.pos(), self.proj))


class Formula_E():

    def __init__(self, x=0, y=0, theta=0, scale=1, framerate=60):
        ### Vehicle Parameters ###
        # Geometry
        self.L = 2.97 * scale # m
        self.length = 5.02 *scale # m (scaled for drawing)
        self.width = 1.70 * scale # m (scaled for drawing)
        self.A = 1.1 # m^2 (frontal Area)
        self.phi_max = 20 * np.pi/180 # rad
        self.phi_sigma = 4 * np.pi/180
        self.scale = scale

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
        self.f_max = self.mu * self.m
        self.brake_max = -self.f_max # Assume the brakes are capable of locking tires at any point
        self.v_cross = self.P_max/self.f_max
        # self.v_max = 320 / 3.6 # kph -> m/s
        

        # Initial Conditions:
        self.state = Car_State(x, y, theta, 0)
        self.phi = 0
        self.dt = 1/framerate

    def rand_control(self, uniform=True):
        if uniform:
            phi = rand.uniform(-self.phi_max, self.phi_max)
            acc = rand.uniform(-1, 1) # Basically throttle/brake percentages, forces are taken care of in f_acc()

        else:
            phi = rand.normal(loc=0, scale=self.phi_sigma, size=1)
            phi = float(np.clip(phi, -self.phi_max, self.phi_max))

            
            acc_sub = rand.normal(loc=0, scale=.15, size=1)
            if acc_sub >= 0:
                acc = 1-float(np.clip(acc_sub, -1, 1))
            else:
                acc = -1-float(np.clip(acc_sub, -1, 1))

        return (acc, phi)

    def theta_dot(self, v, phi): return v/self.L*np.atan(phi)

    def turning_circle(self, phi): return self.L/np.tan(phi)

    def update_pos(self, state:Car_State=None): 
        if not state:
            state = self.state
        theta = state.theta

        state.x += state.v*np.cos(theta)
        state.y += state.v*np.sin(theta)
        state.theta += self.theta_dot(state.v, self.phi)

        theta = state.theta

        r_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta),  np.cos(theta)]])

        car_box =  [
                    r_matrix @ np.array([0,  self.width]) + state.pos(), # Rear left
                    r_matrix @ np.array([0, -self.width]) + state.pos(), # Rear right
                    r_matrix @ np.array([2*self.length, -self.width]) + state.pos(), # Front left
                    r_matrix @ np.array([2*self.length,  self.width]) + state.pos(), # Front right
                   ]
        
        return geom.polygon.Polygon(car_box)
    
    def get_hitbox(self, state:Car_State=None):
        if not state:
            state = self.state
        theta = state.theta
        r_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta),  np.cos(theta)]])

        car_box =  [
                    r_matrix @ np.array([0,  self.width]) + state.pos(), # Rear left
                    r_matrix @ np.array([0, -self.width]) + state.pos(), # Rear right
                    r_matrix @ np.array([2*self.length, -self.width]) + state.pos(), # Front left
                    r_matrix @ np.array([2*self.length,  self.width]) + state.pos(), # Front right
                   ]
        
        return geom.polygon.Polygon(car_box)

    def check_grip(self, v, acc, phi): return self.f_req(v, acc, phi) < self.f_max_grip(v)    
    
    def accelerate(self, f, state:Car_State=None): 
        if not state:
            state = self.state
        state.v += f/self.m * self.dt

    ### FORCES ###
    def f_acc(self, a, state:Car_State=None):
        if not state:
            state = self.state
        """ a = throttle or brake %. Both are kept under the traction limit (as if it has traction control) """
        if a > 0:
            # Automatically reduces power to traction limit if necessary
            if state.v == 0:
                f_acc = self.mu*self.m * a
                a2 = a
            else:
                f_acc = min(self.mu*self.f_n(state.v), abs(self.P_max * a / state.v))
                a2 = f_acc * state.v / self.P_max
        else:
            # This feels a little weird... It's a percent of the available grip not a percent of the 
            # available brake pressure like a driver would normally have. Probably fine, right??
            if state.v > 0:
                f_acc = self.mu*self.f_n(state.v) * a
                a2 = -a
            else:
                f_acc = 0
                a2 = -a
                state.v = 0
            
        return f_acc - self.f_drag(state.v), a2
    
    def f_turn(self, v, phi): return self.m * v**2 / self.turning_circle(phi)

    def f_drag(self, v): return 1050 * (v/50)**2 # N (From Linkedin AirShaper CFD)

    def f_down(self, v): return 1030 * (v/50)**2 # N (From Linkedin AirShaper CFD)

    def f_roll(self, v): return 1000 if v > 0 else 0
       
    def f_max_grip(self, v): return self.mu * self.f_n(v)

    def f_n(self, v): return self.m * self.g + self.f_down(v)

    def f_motor(self, v): return min(self.f_max_grip(v), self.P/v)

    def f_req(self, v, acc, phi): return np.linalg.norm([self.f_acc(acc)[0], self.f_turn(v, phi)])

    def scale_control(self, v, acc, phi):
        f_gas, f_turn = self.calc_forces(v, acc, phi)
        f_mag = np.sqrt(f_gas**2 + f_turn**2)
        self.f_max = self.f_max_grip(v)

        if f_mag < self.max_f:
            return (acc, phi)
        else:
            return (acc/f_mag*self.f_max, np.atan(np.tan(phi)/f_mag*self.f_max)) # FIXME: Check this.

    def calc_forces(self, v, acc, phi):  
        f_acc = self.f_acc(acc)
        f_turn = self.f_turn(v, phi)
        norm = np.linalg.norm([f_acc, f_turn])
        return (f_acc, f_turn, norm)
