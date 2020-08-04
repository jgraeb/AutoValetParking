# ====================== Overview ====================== #
# % ------- Class Gridworld ---------- % #
# Constructing a gridworld with 2 players: system and environment agents and other objects such as static obstacles, static cones.
# + Attributes: 
#   - Nrows: No. of rows in grid
#   - Ncols: No. of columns in grid
#   - Nsys: No. of system agents
#   - Nenv: No. of environment agents
#   - cones: List of all cones on the grid
#   - static_obstacles: List of static obstacles in the grid
#
# + add_player:
#   - Arguments: player_name is the name of the agent, player_type is the 's' or 'e' for system or environment, player_transitions_type is the type of transitions an agent is restricted to take. If player_transitions_type = ['all'], then the agent can occupy any part of the grid
# If player_transitions_type == ['specific', transitions], where transitions is a list of length M*N of the form transitions[ii] = {jj| where jj is the node location on the grid that the agent can transition to}.
# + add_static_obstacles:
#   - Arguments: player_name is the name of the agent, player_type is the 's' or 'e' for system or environment
# + set_states: Cretaes a dictionary of coordinates to numbers representing state ids. Numbering starts from the top left corner of the grid. So cell [1,1] has node_id 1, cell [1,2] has node_id 2, so on ...
# + construct_graph():
# + get_game_graph():
# + plot_win_set():

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

class GridWorld:
    def __init__(self, size):
        self.Nrows = size[0]
        self.Ncols = size[1]
        self.Nsys = 0 
        self.Nenv = 0
        self.cones = []
        self.static_obstacles = []
        self.player_names = []
        self.players = dict()
        self.node2cell = None
        self.cell2node = None
        self.set_states()
        self.G = []
        self.gridT = None
    def get_C2N(self):
        return self.cell2node
    def get_N2C(self):
        return self.node2cell
    def get_player(self, player_name):
        return self.players[player_name]
    def add_player(self, player_name, player_transitions_type, player_type):
        self.player_names.append(player_name)
        self.players[player_name] = pc.Player(player_name, self, player_transitions_type, player_type)

        if(player_type == 's'):
            self.Nsys += 1
        else:
            self.Nenv += 1
    
    def grid_transitions(self):
        cell2node = self.cell2node.copy()
        T = [[] for ii in range(self.Nrows * self.Ncols)]
        for ii in range(1, self.Nrows+1):
            for jj in range(1, self.Ncols+1):
                cell = [ii, jj]
                cell_trans = [[ii, jj]]
                if cell2node[(cell[0], cell[1])] not in self.static_obstacles:
                    if (ii == 1):
                        cell_trans.append([ii+1, jj])
                    if (1 < ii and ii < self.Nrows):
                        cell_trans.append([ii+1, jj])
                        cell_trans.append([ii-1, jj])
                    if (ii == self.Nrows):
                        cell_trans.append([ii-1, jj])
                    if (jj == 1):
                        cell_trans.append([ii, jj+1])
                    if (1 < jj and jj < self.Ncols):
                        cell_trans.append([ii, jj+1])
                        cell_trans.append([ii, jj-1])
                    if (jj == self.Ncols):
                        cell_trans.append([ii, jj-1])
                    transitions = [cell2node[(c[0], c[1])] for c in cell_trans if cell2node[(c[0], c[1])] not in self.static_obstacles]
                    # We need to index by 1.
                    T[cell2node[(cell[0], cell[1])] - 1] = transitions.copy()
                else:
                    T[cell2node[(cell[0], cell[1])] - 1] = [cell2node[(cell[0], cell[1])]]
        self.gridT = T.copy()
    
    def add_static_obstacles(self, sobs):
        snodes = [self.cell2node[tuple(si)] for si in sobs]
        snodes_new = [s for s in snodes if s not in self.static_obstacles]
        self.static_obstacles.extend(snodes_new)
        self.grid_transitions() # Resetting the transition dynamics

    def add_cones(self, c):
        self.cones.extend(c)
    
    def set_states(self):
        node2cell_list = []
        cell2node_list = []
        for ii in range(1, self.Nrows+1):
            for jj in range(1, self.Ncols+1):
                cell = self.Nrows*(ii-1) + jj
                node2cell_list.append((cell, (ii, jj)))
                cell2node_list.append(((ii, jj), cell))
        node2cell = dict(node2cell_list)
        cell2node = dict(cell2node_list)
        self.node2cell = node2cell.copy()
        self.cell2node = cell2node.copy()
    
    def construct_graph(self):
        self.G = gg.GameGraph(self.Nrows, self.Ncols, self.static_obstacles, self.cell2node, self.node2cell, self.players)
    
    def get_game_graph(self):
        return self.G

    def base_plot(self, fignum):
        lw = 2

        fig = plt.figure(fignum)
        ax = plt.gca()

        # Setting up grid and static obstacles:
        # Grid matrix has extra row and column to accomodate the 1-indexing of the gridworld
        grid_matrix = np.zeros((self.Nrows+1, self.Ncols+1))
        # Positioning static obstacles:
        for s in self.static_obstacles:
            scell = self.node2cell[s]
            grid_matrix[scell[0]][scell[1]] = 1

        cmap = ListedColormap(['w', 'r'])
        im = ax.imshow(grid_matrix, cmap=cmap)
        
        # Setting up gridlines
        ax.set_xlim(0.5,self.Ncols+0.5)
        ax.set_ylim(self.Nrows+0.5, 0.5)

        # Labels for major ticks
        ax.set_xticklabels(np.arange(1, self.Ncols+1, 1), minor='True')
        ax.set_yticklabels(np.flipud(np.arange(1, self.Nrows+1, 1)), minor='True')

        # Gridlines based on minor ticks
        ygrid_lines = np.flipud(np.arange(1, self.Nrows+1, 1)) - 0.5
        xgrid_lines = np.arange(1, self.Ncols+1, 1) - 0.5
        
        # Major ticks
        ax.set_xticks(xgrid_lines)
        ax.set_yticks(ygrid_lines)

        # Minor ticks
        ax.set_yticks(ygrid_lines+0.5, minor='True')
        ax.set_xticks(xgrid_lines+0.5, minor='True')
        ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=lw)
        plt.setp(ax.get_xmajorticklabels(), visible=False)
        plt.setp(ax.get_ymajorticklabels(), visible=False)
        return fig, ax, im

    # W: winning set, env_locations are cells on the grid that the environment can occupy
    def plot_win_set(self, W, env_locations, win_agent, min_fig):
        msz  = 12
        nE = len(env_locations)
        lW = len(W)
        W_env_state = [[] for ii in range(nE)]
        Wprev = W[0]
        # Go through W and make lists for winning sets where each environment location is the same.
        for jj in range(nE):
            ei = env_locations[jj]
            W_state = []
            for ii in range(lW):
                Wi = W[ii].copy()
                if(ii>0 and Wi==Wprev):
                    continue
                sys_states = [cell[1] for cell in Wi if cell[0]==ei]
                W_state.append(sys_states)
                Wprev = W[ii].copy()
            W_env_state[jj] = W_state.copy()
        
        # Plotting figures:
        fignum_list = [ii+min_fig for ii in range(nE)]
        im_list = [[] for f in fignum_list]
        ax_list = [[] for f in fignum_list]
        fig_list = [[] for f in fignum_list]

        for ii in range(len(fignum_list)):
            fignum = fignum_list[ii]
            ei = env_locations[ii]
            fig, ax, im = self.base_plot(fignum)
            
            W_cells = W_env_state[ii]
            lW_cell = len(W_cells)
            grid_matrix = np.zeros((self.Nrows+1, self.Ncols+1))

            # Setting colors:
            # Plotting environment regions on grid
            color_factor = 10
            if win_agent == 's':
                color = cm.get_cmap('Blues', 128)
                color_map = color(np.linspace(0,lW_cell,color_factor*lW_cell))
            else:
                color = cm.get_cmap('Greens', 128)
                color_map = color(np.linspace(0,lW_cell,color_factor*lW_cell))
            for Wi_cell in W_cells:
                for cell in Wi_cell:
                    grid_matrix[cell[0]][cell[1]] += color_factor
            cmap = ListedColormap(color_map)
            im = ax.imshow(grid_matrix, cmap=color, alpha=0.8)
            ax.plot(ei[1], ei[0], "g*", markersize=msz) # Plotting environment
            im_list[ii] = im
            ax_list[ii] = ax
            fig_list[ii] = fig
        return fig_list, im_list, ax_list


    # Function to generate static obstacles for a single coverage specification
    # This function of generating static obstacles does so by prev, postv and v labels
    # sys_reach_goal: Set of goal cells corresponding to system specifications
    # cover_goal: States that correspond to coverage of a single atomic proposition
    # G: single_player graph that the search needs to happen on
    def generate_static_obs(self, cover_goal, sys_reach_goal, G):
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
        W0 = [self.cell2node[tuple(g)] for g in sys_reach_goal] 
        coverage_states = [self.cell2node[tuple(g)] for g in cover_goal]
        for v in W0:
            token_dict[v] = tokens[0]
            Value_f[v] = 0
        W = [W0]
        U = [] # No unsafe states in a graph with no moving obstacles
        quantifier = "exists"
        fixpoint = False
        len = 0

        while not fixpoint:
            Wi = W[len]
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
                len+=1
        # Remove repitions:
        static_obs = list(dict.fromkeys(static_obs))
        static_obs = [list(self.node2cell[n]) for n in static_obs]
        return static_obs, token_dict

    # Function that generates static obstacles for multiple coverage specifications:
    # Ex: We're given a list of atomic propositions to cover in reverse order: [psi1, psi2, ..., psin] where psi1 needs to be covered last before system reaches its goal phi, and psin needs to be covered first in the test 
    def gen_static_obs_test_run(self, cover_goals, reach_goal):
        new_static_obs = []
        nC = len(cover_goals)
        self.grid_transitions() # Set transitions with existing static obs, if any
        
        # Single player graph:
        G = nx.DiGraph()
        E = []
        for key, val in self.cell2node.items():
            G.add_node(val)
            for succ in self.gridT[val-1]:
                E.append((val, succ))
        G.add_edges_from(E)
        nodes = list(G.nodes)
        # phi: set of cells that satisfy the lambda function reach_goal
        phi = [self.node2cell[v] for v in nodes if reach_goal(v)]
        for cover_idx in range(1, nC+1):
            # psi: set of cells satisfying a coverage goal, one of which must be covered by adding static obstacles:
            cover_goal = cover_goals[cover_idx-1]
            nodes = list(G.nodes)
            psi = [self.node2cell[v] for v in nodes if cover_goal(v)]
            if (cover_idx > 1):
                cover_goal_prev = cover_goals[cover_idx-2]
                phi = [self.node2cell[v] for v in nodes if cover_goal_prev(v)]
            pdb.set_trace()
            static_obs, token_dict = self.generate_static_obs(psi, phi, G)
            self.add_static_obstacles(static_obs) # Adding static obstacles to the graph
            new_static_obs.append(static_obs)

            # Post-processing: Removing postv tokens from the graph:
            for key, val in token_dict.items():
                if val == "postv":
                    G.remove_node(key)
        return new_static_obs
    # Predecessor function:
    # G: graph
    # U: unsafe states on game graph
    # W0: Winning set
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
    
    # [Deprecate] Plotting Static Obstacle placement:
    def animate_static(self, STATIC_OBS, COVER_GOAL, reach_goal, fignum):
        jmax = 5
        msz = 12
        l = len(COVER_GOAL)
        MAX_FRAMES = l*jmax
        fig, ax, im = self.base_plot(fignum)
        cmap = ListedColormap(['w', 'k', 'r'])
        
        grid_matrix = np.zeros((self.Nrows+1, self.Ncols+1))
        for r in reach_goal:
            grid_matrix[r[0]][r[1]] = 1
        for s in self.static_obstacles:
            scell = self.node2cell[s]
            grid_matrix[scell[0]][scell[1]] = 2
        
        def animate(frame_idx):
            ii = frame_idx//jmax
            jj = frame_idx%jmax
            static_obs = STATIC_OBS[ii]
            coverage_goal = COVER_GOAL[ii]
            for cgoal in coverage_goal:
                ax.plot(cgoal[0], cgoal[1], "k4", markersize=msz)

        ani = animation.FuncAnimation(fig, animate, frames=MAX_FRAMES, interval = 100, blit=True)
        return ani
    
    #  This function is used to animate test runs with the base figure already passed as inputs
    # Argument: skip_transitions could contain list of ['s','e']. This means that the smooth transition part of the animation for that corresponding player 
    # needs to be disabled
    def animate_test_run_gg(self, test_cells, fig, ax, skip_transitions):
        jmax = 10
        msz = 12
        len_TRAJ = len(test_cells)
        points_sys, = ax.plot([], [], "b*", markersize=msz)
        points_env, = ax.plot([], [],"g*",  markersize=msz)
        MAX_FRAMES = jmax*len_TRAJ
        def animate(frame_idx):
            ii = frame_idx//jmax
            jj = frame_idx%jmax
            
            agent_x, agent_y, test_x, test_y = grid_position(ii)
            # In the first iteration, the old_robot_pos is the same as
            # curr_robot_pos
            if ii == 0: 
                old_agent_x = agent_x
                old_agent_y = agent_y
                old_test_x = test_x
                old_test_y = test_y
            else:
                old_agent_x, old_agent_y, old_test_x, old_test_y = grid_position(ii-1)
            if('s' not in skip_transitions):
                int_agent_x = np.linspace(old_agent_x, agent_x, jmax)
                int_agent_y = np.linspace(old_agent_y, agent_y, jmax)
                agent_ypos = int_agent_y[jj]
                agent_xpos = int_agent_x[jj]
            else:
                agent_ypos = old_agent_y
                agent_xpos = old_agent_x
            
            if('e' not in skip_transitions):
                int_test_x = np.linspace(old_test_x, test_x, jmax)
                int_test_y = np.linspace(old_test_y, test_y, jmax)
                test_ypos = int_test_y[jj]
                test_xpos = int_test_x[jj]
            else:
                test_ypos = old_test_y
                test_xpos = old_test_x
            
            points_sys.set_data(agent_ypos, agent_xpos)
            points_env.set_data(test_ypos, test_xpos)
            return [points_sys, points_env]
        
        def grid_position(ii):
            st = test_cells[ii]
            test_st = st[0]
            agent_st = st[1]
            agent_x = agent_st[0]
            agent_y = agent_st[1]
            test_x = test_st[0]
            test_y = test_st[1]
            return agent_x, agent_y, test_x, test_y
        
        ani = animation.FuncAnimation(fig, animate, frames=MAX_FRAMES, interval = 100, blit=True)        
        return ani

    # Plotting test runs:
    # Used for animating game graph test nums
    def animate_test_run(self, test_cells, fignum, fn):
        jmax = 10
        msz = 12
        len_TRAJ = len(test_cells)
        fig, ax, im = self.base_plot(fignum)
        points_sys, = ax.plot([], [], "b*", markersize=msz)
        points_env, = ax.plot([], [],"g*",  markersize=msz)
        MAX_FRAMES = jmax*len_TRAJ
        def animate(frame_idx):
            ii = frame_idx//jmax
            jj = frame_idx%jmax
            
            agent_x, agent_y, test_x, test_y = grid_position(ii)
            # In the first iteration, the old_robot_pos is the same as
            # curr_robot_pos
            if ii == 0: 
                old_agent_x = agent_x
                old_agent_y = agent_y
                old_test_x = test_x
                old_test_y = test_y
            else:
                old_agent_x, old_agent_y, old_test_x, old_test_y = grid_position(ii-1)
            int_agent_x = np.linspace(old_agent_x, agent_x, jmax)
            int_agent_y = np.linspace(old_agent_y, agent_y, jmax)
            int_test_x = np.linspace(old_test_x, test_x, jmax)
            int_test_y = np.linspace(old_test_y, test_y, jmax)
            
            # points_sys.set_data(int_agent_x[jj],int_agent_y[jj])
            # points_env.set_data(int_test_x[jj],int_test_y[jj])
            points_sys.set_data(int_agent_y[jj],int_agent_x[jj])
            points_env.set_data(int_test_y[jj],int_test_x[jj])
            return [points_sys, points_env]
        
        def grid_position(ii):
            st = test_cells[ii]
            test_st = st[0]
            agent_st = st[1]
            agent_x = agent_st[0]
            agent_y = agent_st[1]
            test_x = test_st[0]
            test_y = test_st[1]
            return agent_x, agent_y, test_x, test_y
        
        ani = animation.FuncAnimation(fig, animate, frames=MAX_FRAMES, interval = 100, blit=True)        
        return ani
