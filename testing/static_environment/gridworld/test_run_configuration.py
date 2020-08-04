# Class to define the test configuration at the start of each test run
# Apurva Badithela
# 7/25/20

# Import functions
import numpy as np
import seaborn as sb
from random import randrange
import importlib
import itertools
import pickle
import matplotlib
import matplotlib.animation as animation
from matplotlib import cm
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import os
import math
import networkx as nx
import pdb

import Player_class as pc
import Game_graph_class as gg
import gridworld_class as gwc

class test_run_configuration:
    def __init__(self, gridworld_obj):
        self.GW = gridworld_obj
        self.nrows = gridworld_obj.Nrows
        self.ncols = gridworld_obj.Ncols
        self.node2cell = gridworld_obj.node2cell
        self.cell2node = gridworld_obj.cell2node
        self.nodes = gridworld_obj.node2cell.keys()
        self.transitions = gridworld_obj.gridT
        self.graph = []
        self.final_goal= None
        self.propositions = None
        self.create_graph()
        self.adjacency = np.zeros((gridworld_obj.Nrows+1, gridworld_obj.Ncols+1)) # Adjacency matrix describing where the static obstacles are
        self.static_obstacles = []
        self.cut_transitions = []
    
    def create_graph(self):
        G = nx.DiGraph()
        G.add_nodes_from(self.nodes)
        edge_list = []
        for ii in range(len(self.transitions)):
            end_transitions = self.transitions[ii]
            for jj in end_transitions:
                edge_list.append((ii+1, jj))
        G.add_edges_from(edge_list)
        self.graph.append(G)
    
    # Setting final reach goal:
    def set_final_reach_goal(self, reach_goal):
        self.final_goal = reach_goal
    def set_propositions(self, props):
        self.propositions = props
    # % ======================== Static Obstacle Test Generation Policy ======================================= # %
    # Input cover_goals: Sets of nodes describing the propositions to be covered. Inputs to lambda functions are nodes / node values
    # prop_covered is the set of nodes that has already been covered
    def generate_static_obstacles(self, cover_goals, reach_goal):
        self.propositions = cover_goals.copy()
        self.final_goal = reach_goal.copy()
        lC = len(cover_goals)
        cut_transitions = [] # Contains list of transitions that need to be cut out
        static_obstacles = []
        G = self.graph[0].copy()
        goal = reach_goal.copy()
        props_to_cover = cover_goals.copy()
        while props_to_cover!=[]:
            props_covered, nodes_covered, obs_transitions, static_obs, cut_nodes = self.gen_cut_transitions(G, props_to_cover, goal)
            for v in cut_nodes:
                G.remove_node(v)
            self.graph.append(G)
            cut_transitions.append(obs_transitions)
            static_obstacles.append(static_obs)
            goal = nodes_covered.copy() # Updating what the new goal is
            for l in range(len(props_covered)):
                props_to_cover.remove(props_covered[l])
        return cut_transitions, static_obstacles

    # Main routine for pruning the graph as static obstacles with labels are searched. Key difference from the other method is that all propositions of one kind are considered together
    def generate_static_obstacles_via_labels(self, cover_goals, reach_goal):
        self.propositions = cover_goals.copy()
        self.final_goal = reach_goal.copy()
        lC = len(cover_goals)
        static_obstacles = []
        G = self.graph[0].copy()
        goal = reach_goal.copy()
        props_to_cover = cover_goals.copy()
        for ii in range(lC):
            props_to_cover = cover_goals[ii].copy()
            static_obs, token_dict = self.generate_static_obs_via_labels_subr(G, props_to_cover, goal)
            for key, val in token_dict.items():
                if val == "postv":
                    G.remove_node(key)
            self.graph.append(G)
            static_obstacles.append(static_obs)
            goal = props_to_cover.copy() # Updating what the new goal is
        return static_obstacles

    # Subroutine for synthesizing static obstacles via label: {"postv", "prev", "v"}
    def generate_static_obs_via_labels_subr(self, G, cover_goal, sys_reach_goal):
        tokens = ["postv", "v", "prev"] # Tokens assigned to states after the coverage, coverage states and states that can be forced to pass through v
        static_obs = []

        # Construct single player graph:
        token_dict = dict()
        Value_f = dict()
        E = []
        for val in list(G.nodes):
            token_dict[val] = ""
            Value_f[val] = float('inf')

        # Computing predecessors and assigning tokens:
        W0 = sys_reach_goal.copy()
        coverage_states = cover_goal.copy()
        for v in W0:
            token_dict[v] = tokens[0]
            Value_f[v] = 0
        W = [W0]
        U = [] # No unsafe states in a graph with no moving obstacles
        quantifier = "exists"
        fixpoint = False
        l = 0

        while not fixpoint:
            Wi = W[l]
            pre_W0 = self.pre(G, U, Wi, quantifier)
            # Parsing through pre: 
            for v in pre_W0:
                if v not in Wi:
                    Value_f[v] = len
                    if v in coverage_states:
                        token_dict[v] = tokens[1]
                    else:
                        v_succ = list(G.successors(v))
                        v_succ_Wi = [v for v in v_succ if v in Wi] # All vertices in v_succ_Wi are labeled
                        v_succ_true = [v for v in v_succ_Wi if (token_dict[v]==tokens[1] or token_dict[v]==tokens[2])]
                        v_succ_false = [v for v in v_succ_Wi if (token_dict[v] == tokens[0])]
                        if v_succ_true != []:
                            token_dict[v] = "prev"
                            if v_succ_false!=[]:
                                static_obs.extend(v_succ_false)
                        else:
                            if v_succ_false != []:
                                token_dict[v] = "postv"
            Wi_new = Wi.copy()
            Wi_new.extend(pre_W0)
            Wi_new = list(dict.fromkeys(Wi_new)) # Remove repitions:
            if(Wi == Wi_new):
                fixpoint = True
            else:
                W.append(Wi_new)
                l+=1
        # Remove repitions:
        static_obs = list(dict.fromkeys(static_obs))
        # static_obs = [list(self.node2cell[n]) for n in static_obs]
        return static_obs, token_dict

    # Subroutine for deciding which transitions should be cut:
    def gen_cut_transitions(self, G, props_to_cover, goal):
        cut_nodes = []
        obs_transitions = []
        static_obs = []
        props_to_remove = []
        lP = len(props_to_cover)
        Value_f = dict()
        for val in list(G.nodes):
            Value_f[val] = float('inf')
        W = [goal]
        U = [] # No unsafe states in a graph with no moving obstacles
        quantifier = "exists"
        fixpoint = False
        node_covered = []
        l = 0
        for v in goal:
            Value_f[v] = 0
        Value_stop = []
        pre_W0_only = []

        while not fixpoint:
            Wi = W[l]
            cut_nodes.extend(Wi)
            pre_W0 = self.pre(G, U, Wi, quantifier)
            # Parsing through pre: 
            for v in pre_W0:
                # Updating value function 
                if v not in Wi:
                    Value_f[v] = l+1
                    # Checking if v is a proposition of interest
                    is_v_proposition = [v in p for p in props_to_cover]
                    if any(is_v_proposition):
                        node_covered.append(v)
                        remove_prop = [props_to_cover[ii] for ii in range(lP) if is_v_proposition[ii]]
                        props_to_remove.extend(remove_prop)
                        Value_stop = l+1

            if Value_stop != []:
                static_obs_v_succ = []
                node_covered_succ = []
                for v in node_covered:
                    v_succ = list(G.successors(v))
                    v_succ_lower_Val = [vi for vi in v_succ if Value_f[vi]==Value_f[v]-1]
                    node_covered_succ.extend(v_succ_lower_Val)
                pre_W0_only = [v for v in pre_W0 if (v not in Wi) and (v not in node_covered)] # All the pre's that are not the node of interest
                for v in pre_W0_only:
                    v_succ = list(G.successors(v))
                    cut_v_succ = [vi for vi in v_succ if Value_f[vi] == Value_f[v]-1]
                    for vj in cut_v_succ:
                        obs_transitions.append((v,vj))
                        if vj not in node_covered_succ:
                            static_obs_v_succ.append(vj)
                        else:
                            static_obs_v_succ.append(v)
                static_obs.extend(static_obs_v_succ)
                static_obs = list(dict.fromkeys(static_obs)) # Remove repitions
                # Making sure that successors of the coverage nodes are not blocked:
                # This is redundant
                for v in node_covered:
                    v_succ = list(G.successors(v))
                    for v in v_succ:
                        if v in static_obs:
                            static_obs.remove(v)
                break

            Wi_new = Wi.copy()
            Wi_new.extend(pre_W0)
            Wi_new = list(dict.fromkeys(Wi_new)) # Remove repitions:
            if(Wi == Wi_new):
                fixpoint = True
            else:
                W.append(Wi_new)
                l+=1
        cut_nodes = list(dict.fromkeys(cut_nodes)) # Remove repitions
        obs_transitions = list(dict.fromkeys(obs_transitions)) # Remove repitions
        
        return props_to_remove, node_covered, obs_transitions, static_obs, cut_nodes

    # pre:
    # G: Game graph for the game
    # U: Unsafe states
    # W0: Winning set
    # quantifier: exists or forall. 
    def pre(self, G, U, W0, quantifier):
        pre_W0 = []
        for n0 in W0:
            pred_n0 = list(G.predecessors(n0))
            pred_n0_safe = [p for p in pred_n0 if p not in U]
            if(quantifier == 'exists'):
                pre_W0.extend(pred_n0_safe)
            if(quantifier == 'forall'):
                for pred_n0_ii in pred_n0_safe:
                    pred_n0_ii_succ = list(G.successors(pred_n0_ii))
                    pred_n0_ii_succ_W0 = [p in W0 for p in pred_n0_ii_succ]
                    if all(pred_n0_ii_succ_W0):
                        if pred_n0_ii not in pre_W0:
                            pre_W0.append(pred_n0_ii)
        pre_W0 = list(dict.fromkeys(pre_W0)) # Removes duplicates
        return pre_W0

# % ======================= Updating the matrix representing the static obstacle grid configuration ================= % #
    # Updating the adjacency matrix of where all the static obstacles are located: Locations of static obstacles are indicated by 1
    #  Output: M is a list of np matrices showing the static obstacles added at each step.
    # Multiple goals can be covered at each step or one obstacle is covered at each step
    def update_matrix(self, static_obstacles):
        M = [self.adjacency]
        for ii in range(len(static_obstacles)):
            M_ii = M[ii].copy()
            static_obs = static_obstacles[ii]
            for s in static_obs:
                scell = self.node2cell[s]
                M_ii[scell[0],scell[1]] = 1
            M.append(M_ii)
        self.adjacency = M[-1]
        return M

    # Animate function that shows an animation for how test static obstacles are placed:
    def animate_static(self, fig, ax, im, M):
        jmax = 5
        msz = 12
        l = len(self.propositions)
        MAX_FRAMES = l*jmax
        fig, ax, im = self.base_plot(fig, ax, im, self.propositions, self.final_goal)
        cmap = ListedColormap(['w', 'r'])
        def animate(frame_idx):
            ii = frame_idx//jmax
            jj = frame_idx%jmax
            im.set_array(M[ii])
            return im,
        ani = animation.FuncAnimation(fig, animate, frames=MAX_FRAMES, interval = 100, blit=True)
        return ani

    # Plot function to show base plot with coverage goals:
    def base_plot(self, fig, ax, im, nNodes_to_cover, sys_reach):
        msz = 12
        # Plotting goal:
        for sys_goal_node in sys_reach:
            sys_goal_cell = self.node2cell[sys_goal_node]
            ax.plot(sys_goal_cell[1], sys_goal_cell[0], "ks", markersize=msz) 
        # Plotting coverage goal by different symbols:
        nC = len(nNodes_to_cover)
        cmap = plt.get_cmap('gist_rainbow')
        cNorm  = matplotlib.colors.Normalize(vmin=0, vmax=nC-1)
        scalarMap = cm.ScalarMappable(norm=cNorm, cmap=cmap)
        for ii in range(nC):
            prop_ii = nNodes_to_cover[ii]
            prop_ii_cells = [self.node2cell[p] for p in prop_ii]
            color_val = scalarMap.to_rgba(ii)
            for p in prop_ii_cells:
                ax.plot(p[1], p[0], "X", color= color_val, markersize=msz) 
        return fig, ax, im

    # Plotting function to include static obstacles:
    # Plot function to show base plot with coverage goals:
    def static_obstacle_plot(self, fig, ax, im, obstacle_matrix):
        msz = 12
        nrows = len(obstacle_matrix)
        ncols = len(obstacle_matrix[0])
        for ox in range(nrows):
            for oy in range(ncols):
                if obstacle_matrix[ox,oy]==1:
                    ax.plot(oy, ox, "ro", markersize=msz) 
        return fig, ax, im
    
