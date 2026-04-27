#!/usr/bin/env python
import argparse
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
# plt.ion()
import time
import Track_Collisions
import Formula_E
import shapely.geometry as geom
import shapely.plotting
import shapely


# rand = np.random.default_rng(42)
rand = np.random.default_rng()


# _TRAPPED = 'trapped'
# _ADVANCED = 'advanced'
# _REACHED = 'reached'

class TreeNode:
    '''
    Class to hold node state and connectivity for building an RRT
    '''
    def __init__(self, state:Formula_E.Car_State, cost=0, parent=None):
        self.state = state
        self.cost = cost
        self.children = []
        self.parent = parent

    def add_child(self, child):
        '''
        Add a child node
        '''
        self.children.append(child)

    def __str__(self):
        return f"TreeNode: \n  State: {self.state} \n  Cost:  {self.cost} \n  Children: {self.children} \n  Parent: {self.parent}"

class RRTSearchTree:
    '''
    Searh tree used for building an RRT
    '''
    def __init__(self, init:TreeNode):
        '''
        init - initial tree configuration
        '''
        self.root = init
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, y_query, wx, wc, wt):
        '''
        Find node in tree closets to s_query
        returns - (nearest node, dist to nearest node)
        '''
        min_d = np.inf
        nn = [self.root]
        for n_i in self.nodes:            
            d = np.sqrt(
                          wx * (y_query.state.x - n_i.state.x)**2
                        + wx * (y_query.state.y - n_i.state.y)**2
                        + wc * (y_query.cost   - n_i.cost)**2
                        + wt * (y_query.state.proj - n_i.state.proj)**2
                        )
            if d < min_d:
                min_d = d
                nn.append(n_i)

                if len(nn) > 2:
                    nn.pop(0)

        # print(min_d)
        # input()
        return (nn, min_d)

    def add_node(self, node:TreeNode, parent:TreeNode, traj:np.array):
        '''
        Add a node to the tree
        node - new node to add
        parent - nodes parent, already in the tree
        traj - trajectory connecting child to parent
        '''
        self.nodes.append(node)
        self.edges.append((parent.state, node.state, traj))
        node.parent = parent
        parent.add_child(node)

    def get_states_and_edges(self):
        '''
        Return a list of states and edgs in the tree
        '''
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        '''
        Get the path from the root to a specific node in the tree
        n - node in tree to get path to
        '''
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent
        path.append(n.state)
        path.reverse()
        return path
    
    def remove_node(self, n):
        self.nodes.remove(n) 

class RRT(object):
    '''
    Rapidly-Exploring Random Tree Planner
    '''
    def __init__(self, num_samples:int, track:str, ts_max:int, uniform=False, framerate=60, connect_prob=0.05, window_size=(1200, 800), visualize=False):
        '''
        Initialize an RRT planning instance
        '''
        self.track_name = track
        self.track = Track_Collisions.Track(track, window_size=window_size)
        self.racecar = Formula_E.Formula_E(self.track.start[0], 
                                           self.track.start[1], 
                                           np.atan2(self.track.gradients[0,1], self.track.gradients[0,0]), 
                                           self.track.scale, 
                                           framerate
                                           )


        self.K = num_samples
        self.T_prop_max = ts_max
        self.connect_prob = connect_prob
        self.visualize = visualize
        self.uniform = uniform

        # Expected ranges for normalization
        # self.x_range = window_size[0]
        # self.y_range = window_size[1]
        # self.c_range = self.track.centerline.length * 10 / self.racecar.framerate
        # self.t_range = self.track.centerline.length

        # AO_RRT weight parameters
        self.wx = 1
        self.wc = 5
        self.wt = 1

        self.in_collision = self.track.is_colliding

        # Setup range limits
        self.limits = np.array([[0,0], window_size]).T

        self.path = None

        self.fig, self.ax = plt.subplots()


    def build_ao_rrt(self):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = self.track.goal
        self.init = self.racecar.state
        self.c_max = self.track.centerline.length * 10


        y_min = TreeNode(None, np.inf)
        y_init = TreeNode(self.racecar.state, 0)

        # Build tree and search
        self.T = RRTSearchTree(y_init)

        bad_moves = 0

        # Sample and extend
        for k in range(self.K):

            if rand.random() < self.connect_prob: # Goal Bias?? IDK if AO_RRT uses this
                x_rand = self.track.sample_goal_state()
            else:
                x_rand = self.track.sample_state()

            # x_rand = self.track.sample_state()

            # # Sample random centerline point (not really better...)
            # i_rand = rand.integers(0, len(self.track.centerline.coords))
            # x_rand = shapely.get_point(self.track.centerline, i_rand)
            # x_rand = Formula_E.Car_State(x_rand.xy[0], x_rand.xy[1], self.track.start_ang)

            c_rand = rand.random() * self.c_max
            y_rand = TreeNode(x_rand, c_rand)

            t_rand = rand.integers(1, self.T_prop_max)
            u_rand = self.racecar.rand_control(uniform=self.uniform)
            ys_near, _ = self.T.find_nearest(y_rand, self.wx, self.wc, self.wt)

            for y_near in ys_near:
                pi_new, x_new, steps = self.propagate(y_near.state, u_rand, t_rand)

                if pi_new:
                    c_new = y_near.cost + self.traj_cost(pi_new, steps) 
                    y_new = TreeNode(x_new, c_new)
                    self.T.add_node(y_new, y_near, pi_new)
                    if self.track.goal_reached(self.racecar.get_hitbox(x_new)) and y_new.cost < y_min.cost:
                        y_min = y_new
                        self.c_max = y_min.cost
                        self.path = self.T.get_back_path(y_min)
                        self.prune()
                        if self.visualize:
                            self.plot_track(draw_tree=True, draw_path=True)
                        print(f"Shorter path found on iteration {k}. Length: {self.c_max / self.track.scale}")
                else:
                    bad_moves += 1

            if not k % 250:
                if self.visualize:
                    self.plot_track(draw_tree=True, draw_path=True)
                    self.save_figure(k)
                print(f"{bad_moves}/{k} samples rejected. {k/self.K*100}% Complete")
            
        if self.path != None:
            print(f"Path found! Length: {self.c_max / self.track.scale}")   
            print(f"Nodes in Tree: {len(self.T.nodes)}") 
            self.plot_track(draw_tree=True, draw_path=True) 
            # self.animate_motion()
            return self.T.get_back_path(y_min)
        else:
            print("No path found.")
            self.plot_track(draw_tree=True) 
            return None
        

    def prune(self):
        for n in self.T.nodes:
            if n.cost > self.c_max:
                self.T.remove_node(n)

    def edge_collision(self, edge):
        for e in edge:
            if self.in_collision(e):
                return True
        return False

    def propagate(self, x:Formula_E.Car_State, u=(0, 0), t=10):
        # Forward Dynamics:
        x = x.copy()
        self.racecar.phi = u[1]  # Update car steering angle

        ### NOTE: This just rejects the whole sample if there is any collision
        ### NOTE: It also throws it out if grip is exceeded which can happen mid-path if we're close to the limit
        pi = [x.pos()]

        steps_taken = 0

        for i in range(t):
            if self.racecar.check_grip(x.v, u[0], u[1]): # Friction Circle Check
                f_long, u_1 = self.racecar.f_acc(u[0], x) # Longitudinal forces -> function of v and acc
                
                self.racecar.accelerate(f_long, x) # Update car velocity (in x)
                hitbox = self.racecar.update_pos(x) # Changes values in x
                
                if self.track.is_colliding(hitbox):
                    return (None, None, None)
                elif self.track.goal_reached(hitbox): # Unlikely, but prevents overshooting the goal.
                    pi.append(x.pos())
                    steps_taken += 1
                    break
                else:
                    pi.append(x.pos())
                    steps_taken += 1
                
            else:
                return (None, None, None)
            
        pi = geom.linestring.LineString(pi)
        # print(x.pos())

        return (pi, x, steps_taken)
    
    def traj_cost(self, pi:geom.LineString, steps: int, alpha = 0.001): 
        if not pi or steps is None:
            return None
        else:
            sec = steps / self.racecar.framerate # frames / frames/sec -> sec
            v_avg = pi.length * self.track.scale / sec # m / sec
            if v_avg == 0:
                return 0

            ds = self.track.centerline.project(geom.Point(pi.coords[-1])) - self.track.centerline.project(geom.Point(pi.coords[0]))

            return ds/v_avg

    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False
    
    def plot_track(self, draw_tree=False, draw_path=False):
        self.ax.clear()
        print(self.track.scale)
        track_poly = geom.Polygon(shell=self.track.outer_coords, holes=[self.track.inner_coords])
        # shapely.plotting.plot_polygon(track_poly, ax=ax, facecolor=(.5, .1, .1), edgecolor=(1, 1, 1), add_points=False)
        shapely.plotting.plot_polygon(track_poly, ax=self.ax, add_points=False)
        plt.grid(False)

        if draw_tree:
            for e in self.T.edges:
                shapely.plotting.plot_line(e[2], ax=self.ax, color='black', add_points=False)

        if self.path and draw_path:
            line = geom.LineString([geom.Point(e.pos()) for e in self.path])
            shapely.plotting.plot_line(line, ax=self.ax, color='red', linewidth=1.2, add_points=False)
            
        plt.pause(0.05)

    def save_figure(self, k):
        self.fig.savefig(f"./figs/{self.track_name}_anim/{self.track_name}_{k}.png")

    def animate_motion(self):
        for i, v in enumerate(self.path):
            self.plot_track(False, True)
            shapely.plotting.plot_polygon(self.racecar.get_hitbox(v), ax=self.ax, color='blue', add_points=False)
            self.save_figure(i)



def test_rrt_env(num_samples=500, track='IMS', uniform=False, visualize=False, step_length=10, framerate=60, connect_prob=0.05):
    start_time = time.time()

    rrt = RRT(num_samples, track, step_length, uniform, framerate, connect_prob, visualize=visualize)
    
    plan = rrt.build_ao_rrt()


    run_time = time.time() - start_time
    # print('plan:', plan)
    print( 'run_time =', run_time)

    plt.show() # Keep the plot open after finishing

    
    return plan, rrt

def main():
    parser = argparse.ArgumentParser(description="Rapidly Exploring Random Trees")

    parser.add_argument(
        "-N", "--num_samples",
        type=int,
        default=500,
        help="Number of samples",
    )

    parser.add_argument(
        "--track",
        type=str,
        default="IMS",
        help="Track name",
    )

    parser.add_argument(
        "--step_length",
        type=int,
        default=5,
        help="Step length for propagation",
    )

    parser.add_argument(
        "--framerate",
        type=int,
        default=60,
        help="Framerate for simulation",
    )

    parser.add_argument(
        "--connect_prob",
        type=float,
        default=0.05,
        help="Connection probability",
    )

    parser.add_argument(
        "--uniform", "-u",
        action="store_true",
        help="Force Uniform control sampling",
    )

    parser.add_argument(
        "--visualize", "-v",
        action="store_true",
        help="Visualize track, tree, and plan",
    )

    args = parser.parse_args()
    kwargs = {}
    kwargs['num_samples'] = args.num_samples
    kwargs['track'] = args.track
    kwargs['step_length'] = args.step_length
    kwargs['framerate'] = args.framerate
    kwargs['uniform'] = args.uniform
    kwargs['visualize'] = args.visualize
    kwargs['connect_prob'] = args.connect_prob
    
    plan, rrt = test_rrt_env(**kwargs)

if __name__== "__main__":
    main()
