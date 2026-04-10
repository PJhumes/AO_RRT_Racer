# AO_RRT_Racer

## Track_Collisions.py

Instantiate by calling `track = Track_Collisions.Track("IMS", window_size=window_size)` with an optional `angle` parameter to rotate the track. `angle` is taken care of automatically for IMS, Monza, and Silverstone. This is the object that holds the track data.

### Properties:
centerline: Center line of the track. Taken from GPS data by TU Munich students.
outer_coords: Outer track boundary coordinates.
inner_coords: Inner track boundary coordinates. 
gradients: dy/dx of the track centerline. Can be used to calculate how far the car is pointing away from the centerline vector. 
scale: Track scale for visualization purposes. Although it currently is used in the sim... (FIXME)
prepared: Track "ring" including inner and outer coordinates. Optimized for fast collision checking with racecar.hitbox.
start: xy position of the starting line of the track
start_ang: The angle of the starting line so the car faces down the track to begin.
goal region: The region directly behind the car when it starts. Call track.sample_goal() to sample within this region.

### Functions available:

`load_track(track_name:str, angle:float(optional)):` -> track_data outlined above. Used internally.

`is_colliding(hitbox:geom.Polygon):` -> Boolean if the car is (at least partially) off the track.

`goal_reached(hitbox:geom.Polygon):` -> Boolean if the car is intersecting with the goal region.

`sample_goal():` -> geom.Point within the goal region.

`sample_state():` -> geom.Point within the window (world).


## Formula_E.py

Object containing the dynamics and physical constraints of a Formula E Gen 3 Evo Racecar. Based on basic testing, the dynamics turned out pretty accurate. We're within 2 m/s of the published top speed and within measurement noise of me hitting stop on a stopwatch of the 0-60 mph (26 ish m/s) time based on our simulation parameters. (FIXME: We need to make sure the sim scale and dynamics scale are the same). Create an instance by calling: `Formula_E.Formula_E(x, y, theta, scale, framerate)` where **x, y,** and **theta** are initial conditions.

### Properties:

#### Geometry
self.L: Wheelbase. Used for Bicycle Motion Model
self.length: Vehicle Length (visualization only)
self.width: Vehicle Width (visualization only)
self.A: Vehicle Frontal Area
self.phi_max: Max steering angle
self.scale: Scale factor (FIXME: Ensure Proper Implementation)

#### Tires
self.mu: Tire friction coefficient estimate. 1.8 for full slicks, 1.0 for road tires. Not published data.
self.r_front: Front tire diameter (unused)
self.r: Rear tire diameter (unused)

#### Inertia
self.m: Vehicle mass including driver
self.h: C.o.G. height (unused)

#### Aerodynamics
self.C_D: Drag coeff (unused - calculated using LinkedIn FEA post data)
self.C_L: Lift (downforce) coeff (unused - calculated using LinkedIn FEA post data)
self.rho: Air density at sea level
self.g: gravity
self.C_rr: Rolling Resistance (unused)

#### Powertrain
self.P_max: Max power: 300 kW
self.f_max: Max grip available from the tires at zero velocity (no downforce)
self.brake_max: Max braking force, assume the brakes are capable of locking tires at any point
self.v_cross: Crossover velocity for traction -> power limited (unused)
self.v_max: Max velocity (unused)


#### Initial Conditions:
self.state = self.car_state(x, y, theta)
    state.x = x posiiton
    state.y = y position
    state.theta = z roation (orientation
    )
self.phi: Steering angle
self.v: Forward velocity
self.dt: Simulation timestep - 1/framerate


### Functions Available:
`rand_control(self):` -> Random control action (acc, phi). Brake/gas in range (-1, 1) for pedal percentages and phi (-phi_max, phi_max)

`theta_dot(self, v, phi):` -> Bicycle motion model angular velocity

`update_pos(self):` -> Euler update given current velocity parameters
    
`get_hitbox(self):` -> Get the current hitbox of the car (no update)
    
`accelerate(self, f):` -> Apply throttle/brake force to the vehicle

`f_acc(self, a):` -> Calculate the acceleration force available. Power-limited at high speed and Grip limited at low speed.
    
`f_turn(self, phi):` (Not implemented)

`f_drag(self, v):` -> Calculate air resistance (From Linkedin AirShaper CFD)

`f_down(self, v):` -> Calculate downforce (From Linkedin AirShaper CFD)

`f_roll(self, v):` -> Ballpart Coulomb rolling friction to slow the car down when coasting. Total ballpark guess.
    
`f_max_grip(self, v):` -> Max grip available including downforce

`f_n(self, v):` -> Normal force. Mass*gravity + downforce

`f_motor(self, v):` -> Power used by the motor (FIXME: divide by zero error at zero velocity)

`scale_control(self, v, acc, phi):` -> Shift the controls to stay within the friction circle (We decided to probably not use this right now)

`calc_forces(self, v, acc, phi):` -> Calculates the forces for a given velocity and control input. Compare to friction circle for sampling rejection.


## AO_RRT.py
This is an incomplete implementation of the AO_RRT algorithm using the above objects. It is far from complete, but has the initialization how I initially think wouuld work well. Feel free to use it as a reference. Or fully discard it lol. We *should not* have to use Dr. Hermans' collisions.py file. the Track_Collisions should take care of that for us I think.



## dynamics_test.py
A simple pygame visualization of the track and dynamics. May be useful for visualizing/simulating the final paths. For now, to play the game, clone the repo and then run dynamics_test.py. You may need to install libraries used (Pygame most likely). Arrow keys to drive, up is gas, down is brake, and left and right correspond to turning. It's pretty finnicky right now, but it will get better once scaling is right and dynamics are improved.
