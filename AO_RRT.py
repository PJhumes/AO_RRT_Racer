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
    def __init__(self, num_samples:int, track:str, framerate=60, epsilon=1, connect_prob=0.05, window_size=(1200, 800)):
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
        # self.n = frame_steps
        self.epsilon = epsilon # number of timesteps for a move
        self.connect_prob = connect_prob
        self.wx = 1
        self.wc = 1


        self.in_collision = self.track.is_colliding
        # if collision_func is None:
        #     self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = np.array([[0,0], [window_size]]).T
        print(self.limits)


        # self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_ao_rrt(self, init, goal, T_prop, c_max, U=None):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False
        self.c_max = c_max
        self.goal_region = .25

        y_min = TreeNode(None, np.inf)
        y_init = TreeNode(np.array(init), 0)

        # Build tree and search
        self.T = RRTSearchTree(y_init)

        # Sample and extend
        for k in range(self.K):

            x_rand = self.sample()
            c_rand = rand.random() * self.c_max
            y_rand = TreeNode(x_rand, c_rand)

            t_rand = rand.random() * T_prop
            # u_rand = self.rand_control()
            y_near, _ = self.T.find_nearest(y_rand, self.wx, self.wc) # FIXME

            u_rand = (x_rand - y_near.state) / norm((x_rand-y_near.state)) * self.epsilon
            pi_new, x_new = self.propagate(y_near.state, u_rand, t_rand)
            c_new = y_near.cost + self.traj_cost(pi_new)

            if not self.edge_collision(pi_new):
                y_new = TreeNode(x_new, c_new)
                self.T.add_node(y_new, y_near, pi_new)
                if norm(y_new.state - self.goal) < self.goal_region and y_new.cost < y_min.cost:
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

    def rand_control(self):
        maxv = 5

        rand = self.sample()
        rand /= norm(rand)

        u_rand = rand * 2*maxv - np.ones_like(self.goal) * maxv

        return u_rand

    def edge_collision(self, edge):
        for e in edge:
            if self.in_collision(e):
                return True
        return False

    def propagate(self, x, u, t):
        # Forward Dynamics:
        # Assuming direct velocity control in any direction
        traj_num = max(50, t/self.epsilon*10)
        dt = np.linspace(0, t, traj_num, endpoint=True)
        dx = np.outer(dt, u)

        pi = np.outer(np.ones(traj_num), x) + dx

        for i in range(len(dt)):

            if norm(pi[i, :] - self.goal) < self.goal_region:
                pi = pi[:i+1, :]
                break 
                
        return (pi, pi[-1, :])
    
    def steer(self, x, u, t):
        # Forward Dynamics:
        # Assuming direct velocity control in any direction
        traj_num = max(100, t/self.epsilon*10)
        dt = np.linspace(0, t, traj_num, endpoint=True)
        dx = np.outer(dt, u)

        pi = np.outer(np.ones(traj_num), x) + dx

        return (pi, pi[-1, :])
    
    def traj_cost(self, pi):
        # Simple distance cost for moving in straight lines.
        return norm(pi[0, :] - pi[-1, :])

    def sample(self, no_goal=False):
        '''
        Sample a new configuration
        Returns a configuration of size self.n bounded in self.limits
        '''
        # Return goal with connect_prob probability
        if rand.random() < self.connect_prob and not no_goal:
            return self.goal
        else:
            s = []
            for i in range(self.n):
                diff = self.limits[i, 1] - self.limits[i, 0]
                s.append(self.limits[i, 0] + rand.random() * diff)
            s = np.array(s)
            return s

    def extend(self, T: RRTSearchTree, q):
        '''
        Perform rrt extend operation.
        q - new configuration to extend towards
        returns - tuple of (status, TreeNode)
           status can be: _TRAPPED, _ADVANCED or _REACHED
        '''
        q_near, q_dist = T.find_nearest(q)
        q_new = TreeNode(q_near.state + min(1, self.epsilon/q_dist) * (q - q_near.state), q_near)
        if self.visible(q_near.state, q_new.state):
            T.add_node(q_new, q_near)
            if (q_new.state == q).all():
                return (_REACHED, q_new)
            else:
                return (_ADVANCED, q_new)
        return (_TRAPPED, None)
    
    def extend_connect(self, T: RRTSearchTree, q):
        '''
        Perform rrt-connect extend operation.
        repeatedly calls extend().
        q - new configuration to extend towards
        returns - tuple of (status, TreeNode)
           status can be: _TRAPPED, _ADVANCED or _REACHED
        '''
        while True:
            status = self.extend(T, q)
            if status[0] != _ADVANCED:
                return status


    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

    def visible(self, q_near: np.array, q_new: np.array):
        ''' Returns whether a state q_new is visible from its nearest neighbor state q_near. 
        (Evaluates the edge for collision)
        '''
        vec = q_new-q_near
        res = min(0.1, self.epsilon/10)
        for m in np.arange(1, 0, -res): # Start at the new config (don't bother checking the rest if the end is in collision)
            if self.in_collision(q_near + m*vec):
                return False
        return True

def test_rrt_env(num_samples=500, step_length=5.0, env='./env0.txt',
                 connect=False, bidirection=False, ao=False, t_prop = 1, connect_prob=0.05, rand_map=False):
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = connect_prob,
              collision_func=pe.test_collisions)
    
    if rand_map:
        pe.start = rrt.sample(no_goal=True)
        pe.goal = rrt.sample(no_goal=True)

    if bidirection and connect:
        print('Warning both --bidirection and --connect are set. Executing Bi-directional RRT-Connect')
        
    if bidirection:
        plan = rrt.build_bidirectional_rrt_connect(pe.start, pe.goal)
    elif connect:
        plan = rrt.build_rrt_connect(pe.start, pe.goal)
    elif ao:
        plan = rrt.build_ao_rrt(pe.start, pe.goal, t_prop, 350)
    else:
        plan = rrt.build_rrt(pe.start, pe.goal)

    run_time = time.time() - start_time
    # print 'plan:', plan
    print( 'run_time =', run_time)

    # Setting dynamic_tree and dynamics_plan to True, animates the plot
    # and plans respectively. it is a blocking call, and takes a while
    # to finish, so use only with low samples.
    pe.draw_env(show=False)
    pe.draw_plan(plan, rrt,
                 dynamic_tree=False,
                 dynamic_plan=True,
                 show=True)

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
