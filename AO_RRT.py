#!/usr/bin/env python
import argparse
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
import Track_Collisions
import Formula_E
import shapely.geometry as geom


# rand = np.random.default_rng(42)
rand = np.random.default_rng()


# _TRAPPED = 'trapped'
# _ADVANCED = 'advanced'
# _REACHED = 'reached'

class TreeNode:
    '''
    Class to hold node state and connectivity for building an RRT
    '''
    def __init__(self, state, cost=0, parent=None):
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

    def find_nearest(self, y_query, wx, wc): # FIXME: Make sure this is updated as shown in part IV of the paper.
        '''
        Find node in tree closets to s_query
        returns - (nearest node, dist to nearest node)
        '''
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:            
            d = np.sqrt(wx*norm(y_query.state - n_i.state)**2 + wc*(y_query.cost - n_i.cost)**2)
            if d < min_d:
                nn = n_i
                min_d = d
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
    def __init__(self, num_samples:int, track:str, ts_max:int, framerate=60, connect_prob=0.05, window_size=(1200, 800)):
        '''
        Initialize an RRT planning instance
        '''
        self.track = Track_Collisions.Track(track, window_size)
        self.racecar = Formula_E.Formula_E(self.track.start[0], 
                                           self.track.start[1], 
                                           np.atan2(self.track.gradients[0,1], self.track.gradients[0,0]), 
                                           self.track.scale, 
                                           framerate
                                           )


        self.K = num_samples
        self.T_prop_max = ts_max
        self.connect_prob = connect_prob

        # AO_RRT weight parameters
        self.wx = 1
        self.wc = 1

        self.in_collision = self.track.is_colliding

        # Setup range limits
        self.limits = np.array([[0,0], [window_size]]).T
        print(self.limits)


        self.found_path = False

    def build_ao_rrt(self, T_prop:int, c_max:float):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.T_prop = T_prop
        self.goal = self.track.goal
        self.init = self.racecar.state
        self.c_max = c_max


        y_min = TreeNode(None, np.inf)
        y_init = TreeNode(np.array(self.init), 0)

        # Build tree and search
        self.T = RRTSearchTree(y_init)

        # Sample and extend
        for k in range(self.K):

            # if rand.random() < self.connect_prob: # Goal Bias?? IDK if AO_RRT uses this
            #     x_rand = self.track.sample_goal_state()
            # else:
            #     x_rand = self.track.sample_state()

            x_rand = self.track.sample_state()

            c_rand = rand.random() * self.c_max
            y_rand = TreeNode(x_rand, c_rand)

            t_rand = rand.integers(1, T_prop)
            u_rand = self.racecar.rand_control()
            y_near, _ = self.T.find_nearest(y_rand, self.wx, self.wc) # FIXME Is this right??

            pi_new, x_new = self.propagate(y_near.state, u_rand, t_rand)
            c_new = y_near.cost + self.traj_cost(pi_new)

            if pi_new:
                y_new = TreeNode(x_new, c_new)
                self.T.add_node(y_new, y_near, pi_new)
                if self.track.goal_reached(self.racecar.get_hitbox(x_new)):
                    y_min = y_new
                    self.c_max = y_min.cost
                    self.found_path = True
                    self.prune()
                    print(f"Shorter path found on iteration {k}. Length: {self.c_max}")
            
        if self.found_path:
            print(f"Path found! Length: {self.c_max}")   
            print(f"Nodes in Tree: {len(self.T.nodes)}")  
            return self.T.get_back_path(y_min)
        else:
            print("No path found.")
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
        pi = [x.pos]
        for i in range(len(t)):
            if self.racecar.check_grip(x.v, u[0], u[1]): # Friction Circle Check
                f_long = self.racecar.f_acc(u[0], x) # Longitudinal forces -> function of v and acc
                
                self.racecar.accelerate(f_long, x) # Update car velocity (in x)
                hitbox = self.racecar.update_pos(x) # Changes values in x
                
                if self.track.is_colliding(hitbox):
                    return (None, None)
                else:
                    pi.append(x.pos)
                
            else:
                return (None, None)

        pi = geom.linestring.LineString(pi)
        return (pi, x)
    
    def traj_cost(self, pi:geom.LineString): # FIXME: update to include path line integral cost for optimizing TIME not distance.
        if not pi:
            return None
        else:
            return pi.length # polyline length is path cost for now.

    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False


def test_rrt_env(num_samples=500, track='IMS', step_length=10, framerate=60, connect_prob=0.05):
    # pe = PolygonEnvironment()
    # pe.read_env(env)

    # dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples, track, step_length, framerate, connect_prob)
    
    plan = rrt.build_ao_rrt()


    run_time = time.time() - start_time
    # print 'plan:', plan
    print( 'run_time =', run_time)

    
    return plan, rrt

def main():
    parser = argparse.ArgumentParser(description="Rapidly Exploring Random Trees")

    parser.add_argument(
        "--env",
        type=str,
        default="./env0.txt",
        help="Path to env file",
    )

    parser.add_argument(
        "-N", "--num_samples",
        type=int,
        default=200,
        help="Number of samples",
    )

    parser.add_argument(
        "--epsilon",
        type=float,
        default=1,
        help="Step length for straight line planner",
    )

    parser.add_argument(
        "--goal_bias",
        type=float,
        default=0.05,
        help="Probability that the sampler returns the goal",
    )

    parser.add_argument(
        "--t_prop",
        type=float,
        default=0.05,
        help="Probability that the sampler returns the goal",
    )

    parser.add_argument(
        "--bidirectional",
        action="store_true",
        help="Use Bi-directional RRT-Connect (This takes precedance over connect)",
    )

    parser.add_argument(
        "--connect",
        action="store_true",
        help="Use RRT-Connect (Ignored if bidirectional is set)",
    )

    parser.add_argument(
        "--ao",
        action="store_true",
        help="Use AO-RRT",
    )

    parser.add_argument(
        "--random",
        action="store_true",
        help="Set random start and goal positions",
    )

    args = parser.parse_args()
    kwargs = {}
    kwargs['num_samples'] = args.num_samples
    kwargs['step_length'] = args.epsilon
    kwargs['env'] = args.env
    kwargs['bidirection'] = args.bidirectional
    kwargs['connect'] = args.connect
    kwargs['ao'] = args.ao
    kwargs['rand_map'] = args.random
    kwargs['connect_prob'] = args.goal_bias
    kwargs['t_prop'] = args.t_prop
    
    test_rrt_env(**kwargs)

if __name__== "__main__":
    main()
