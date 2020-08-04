# ======================= Overview ======================= #
# % ------------ Class GameGraph ------------ % #
# Modified game graph class for general environment and system transitions:
# sys_transitions and env_transitions are dictionaries that map system and environment nodes to the test strategy
# Unsafe are pairs of system and environment states that are not safe
# static_obstacles are passed in terms of system states

import numpy as np
import random
from random import randrange
import importlib
import itertools
import pickle
import matplotlib
from matplotlib import cm
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import os
import math
import networkx as nx
import pdb

import gridworld_class 
import Player_class

class GeneralGameGraph:
    def __init__(self, static_obstacles, sys_transitions, env_transitions, unsafe):
        self.G = nx.DiGraph()
        self.static_obs = static_obstacles.copy()
        self.static_grid = nx.Graph() # Graph based on just the grid and not the 2-player game graph with any of the players involved
        self.env_pEdges = env_transitions.copy()
        self.sys_pEdges = sys_transitions.copy()
        self.Ns = len(sys_transitions)
        self.Ne = len(env_transitions)
        self.V = None
        self.Vweight = None # Dictionary mapping vertices to their weights
        self.E = None
        self.U = None
        self.vertices()     
        self.unsafe_states(unsafe)
        self.edges() # This sets the edges based on the data collected from vertices.
        self.G.add_edges_from(self.E)
        self.G.add_nodes_from(self.V)
        self.set_vertex_weight()
        self.Val_sys, self.Val_sys_set = self.init_val_function('s') # Value function corresponding to system objective
        self.Val_env, self.Val_env_set = self.init_val_function('e') # Value function corresponding to environment objective

    def unpack_vertex(self, v):
        if(v[0:2]=="v1"):
            p = 'e'
        else:
            p = 's'
        ne, ns = self.state2node(self.Ns, self.Ne, int(v[3:]))
        return p, ns, ne
    # args are a dictionary of lambda functions and the coresponding weight to be associated with states at which those lambda functions are true
    def set_vertex_weight(self, *args):
        Vweight_list = [[v, 0] for v in self.V]
        self.Vweight = dict(Vweight_list)
        if args:
            for lambda_func, weight in args[0].items():
                for v in self.V:
                    p, ns, ne = self.unpack_vertex(v)
                    if (v[0:2] == "v1" and lambda_func(ns, ne)): # Weight is only given to the environment state
                        self.Vweight[v] = weight

    def init_val_function(self, ptype):
        Val_list = []
        Val_set_list = []
        if ptype == 's':
            base = 'inf'
        elif ptype == 'e':
            base = '0'
        else:
            print("Error: input argument pytpe must be 's' or 'e'")
        for vi in self.V:
            Val_list.append([vi, float(base)])
            Val_set_list.append([vi, 0]) # Once set to 1, then it doesn't change
        Val = dict(Val_list)
        Val_set = dict(Val_set_list)
        return Val, Val_set
    
    def get_value_function(self, ptype):
        if ptype == 's':
            return self.Val_sys
        elif ptype == 'e':
            return self.Val_env
        else:
            print("Err: input argument must be 's' or 'e'")
    
    # This function defines what transitions are allowed from each grid cell
    def state(self, Ns, Ne, ns, ne):
        return Ne*(ns - 1) + ne

    def state2node(self, Ns, Ne, st):
        if st%Ne == 0:
            env_state = Ne
            sys_state = st/Ne 
        else:
            env_state = st%Ne
            sys_state = st//Ne + 1
        return env_state, sys_state

    def shortest_distance(self, bridge, start):
        dist_to_bridge = []
        for b in bridge:
            path_b = nx.dijkstra_path(self.static_grid, start, b)
            dist_to_bridge.append(len(path_b))
        return dist_to_bridge

    def edges(self):
        env_edges = self.env_pEdges.copy()
        sys_edges = self.sys_pEdges.copy()
        Edges = []
        for ns in range(1, self.Ns+1):
            for ne in range(1, self.Ne+1):
                start_state = self.state(self.Ns, self.Ne, ns, ne)
                env_transitions = env_edges[ne-1]
                sys_transitions = sys_edges[ns-1]
                end_states_env = [self.state(self.Ns, self.Ne, ns, ne_end) for ne_end in env_transitions]
                end_states_sys = [self.state(self.Ns, self.Ne, ns_end, ne) for ns_end in sys_transitions]
                vstart_sys = "v2_"+str(start_state)
                vstart_env = "v1_"+str(start_state)
                
                # Make assertions to check if vertices are in V:
                assert(vstart_env in self.V)
                assert(vstart_sys in self.V)

                # Environment to system transitions:
                if vstart_env not in self.U:
                    for end_state in end_states_env:
                        vend_sys = "v2_"+str(end_state)
                        Edges.append((vstart_env, vend_sys))
                
                # System to environment transitions:
                if vstart_sys not in self.U:
                    for end_state in end_states_sys:
                        vend_env = "v1_"+str(end_state)
                        Edges.append((vstart_sys, vend_env))
        self.E = Edges.copy() # Setting the edges variable
    # Function to remove certain edges from the graph as the transition has been constructed:
    # edge_node_pairs = [["v1_2", "v2_2"], .. ] is the list of directional edges that has to be removed
    def remove_edges(self, edges_node_pairs):
        for node_pairs in edges_node_pairs:
            start = node_pairs[0]
            end = node_pairs[1]
            self.G.remove_edge(start, end)

    def vertices(self):
        V= []
        for ns in range(1, self.Ns+1):
            for ne in range(1, self.Ne+1):
                s = self.state(self.Ns, self.Ne, ns, ne)
                V.extend(["v1_"+str(s)]) # Environment action vertices
                V.extend(["v2_"+str(s)]) # System action vertices
        self.V = V

    def get_edges_vertices(self):
        return self.E, self.V

# Make this function automatic non-graph case
# This function computes the set of states that are unsafe, i.e when system collides with static obstacles or with the moving environment
    def unsafe_states(self, unsafe):
        unsafe_states = []
        # Unsafe states from system being in the same state as a static obstacle:
        for ns in self.static_obs:
            for ne in range(1, self.Ne+1):
                s = self.state(self.Ns, self.Ne, ns, ne)
                unsafe_states.extend(["v1_"+str(s)]) # Environment action vertices
                unsafe_states.extend(["v2_"+str(s)]) # System action vertices

        # Unsafe states from system and environment being in the same state:
        for unsafe_combo in unsafe:
            ne = unsafe_combo[0]
            ns = unsafe_combo[1]
            s = self.state(self.Ns, self.Ne, ns, ne)
            unsafe_states.extend(["v1_"+str(s)]) # Environment action vertices
            unsafe_states.extend(["v2_"+str(s)]) # System action vertices
        # Unsafe
        self.U = unsafe_states.copy()

    def get_unsafe_states(self):
        return self.U

# This function is the predecessor operator that computes the set of states from which for all
#   + pre: Predecessor computation on a game graph
#   + W0: Winning set that the transition must end up in
#   + ptype: 's' or 'e' for system or environment, i.e whether the pre set should be comprised of system or environment states
#   + quantifier: 'exists' or 'forall'
    def pre(self, W0, ptype, quantifier):
        pre_W0 = []
        for n0 in W0:
            pred_n0 = list(self.G.predecessors(n0))
            pred_n0_safe = [p for p in pred_n0 if p not in self.U]
            if(quantifier == 'exists'):
                pre_W0.extend(pred_n0_safe)
            if(quantifier == 'forall'):
                for pred_n0_ii in pred_n0_safe:
                    pred_n0_ii_succ = list(self.G.successors(pred_n0_ii))
                    pred_n0_ii_succ_W0 = [p in W0 for p in pred_n0_ii_succ]
                    if all(pred_n0_ii_succ_W0):
                        if pred_n0_ii not in pre_W0:
                            pre_W0.append(pred_n0_ii)
        pre_W0 = list(dict.fromkeys(pre_W0)) # Removes duplicates
        return pre_W0
# Similar to the previous function pre but with the extra condition on the System value function
    def pre2(self, W0, ptype, quantifier, win_agent):
        pre_W0 = []
        for n0 in W0:
            pred_n0 = list(self.G.predecessors(n0))
            pred_n0_safe = [p for p in pred_n0 if p not in self.U]
            pred_n0_safe_nodes=[]
            for v in pred_n0_safe:
                p, ns, ne = self.unpack_vertex(v)
                pred_n0_safe_nodes.append([ne, ns])
            if(win_agent=='e' and ptype == 's'): # If the winning set is for the environment and the player in the pre set are system action states, then:
                pred_n0_safe = [p for p in pred_n0_safe if(self.Val_sys[n0] <= self.Val_sys[p])]
            if(quantifier == 'exists'):
                pre_W0.extend(pred_n0_safe)
            if(quantifier == 'forall'):
                for pred_n0_ii in pred_n0_safe:
                    pred_n0_ii_succ = list(self.G.successors(pred_n0_ii))
                    pred_n0_ii_succ_W0 = [p in W0 for p in pred_n0_ii_succ]
                    if all(pred_n0_ii_succ_W0):
                        if pred_n0_ii not in pre_W0:
                            pre_W0.append(pred_n0_ii)
        pre_W0 = list(dict.fromkeys(pre_W0)) # Removes duplicates
        return pre_W0

# This function needs to be called only after the initial value function has been set for each of the respective agents
# Sets the value function for the system and the environment depending on their quantifiers. For the system, the quantifier does not change the value function. The environment might have different ways of setting the value function depending on the quantifier.
# TODO: Change how the environemnt value function is set.
# TODO: If the winning agent is the environment, the value function for the system states do not change no matter if the quantifier is forall or exists
# determine_environment_value2 is the same as environment_value except that it includes a more refined notion of the winning set.
# determine_environment_value_1 is the most relevant one...
    def determine_environment_value3(self, W0, quant, ptype, win_agent):
        assert(win_agent == 'e')
        if ptype == 's':
            if (quant == 'forall'):
                for v in W0:
                    if(self.Val_env_set[v] == 0):
                        self.Val_env_set[v] = 1
                        successors = list(self.G.successors(v))
                        Val_successors = [self.Val_env[s] for s in successors]
                        max_successor = successors[Val_successors.index(max(Val_successors))]
                        self.Val_env[v] = self.Val_env[max_successor]
            if (quant == 'exists'):
                for v in W0:
                    if(self.Val_env_set[v] == 0):
                        self.Val_env_set[v] = 1
                        successors = list(self.G.successors(v))
                        Val_successors = [self.Val_env[s] for s in successors]
                        max_successor = successors[Val_successors.index(max(Val_successors))]
                        self.Val_env[v] = self.Val_env[max_successor]
        if ptype == 'e':
            for v in W0:
                if(self.Val_env_set[v] == 0):
                    self.Val_env_set[v] = 1
                    successors = list(self.G.successors(v))
                    if successors != []:
                        Val_successors = [self.Val_env[s] for s in successors]
                        max_successor = successors[Val_successors.index(max(Val_successors))]
                        self.Val_env[v] = self.Vweight[v] + self.Val_env[max_successor]

    def determine_environment_value2(self, W0, quant, ptype, win_agent):
        pre_win_set = []
        assert(win_agent == 'e')
        if ptype == 's':
            if (quant == 'forall'):
                for v in W0:
                    if(self.Val_env_set[v] == 0):
                        self.Val_env_set[v] = 1
                        successors = list(self.G.successors(v))
                        Val_successors = [self.Val_env[s] for s in successors]
                        Val_sys_successors = [self.Val_sys[s] for s in successors]
                        Val_sys_cur = self.Val_sys[v]
                        # write a line about potential states that are actually good with just there exists..
                        Val_sys_dec = [s for s in Val_sys_successors if (s < Val_sys_cur)]
                        Val_sys_dec_idx = [ii for ii in range(len(Val_sys_successors)) if (Val_sys_successors[ii] < Val_sys_cur)]
                        Val_env_dec = [Val_successors[ii] for ii in Val_sys_dec_idx]
                        if Val_sys_dec:
                            pre_win_set.append(v)
                            min_succ_Val_env_dec = Val_env_dec.index(min(Val_env_dec))
                            min_ii = Val_sys_dec_idx[min_succ_Val_env_dec]
                            min_successor = successors[min_ii]
                            # min_successor = successors[Val_successors.index(min(Val_successors))]
                            self.Val_env[v] = self.Val_env[min_successor]
            if (quant == 'exists'):
                for v in W0:
                    if(self.Val_env_set[v] == 0):
                        self.Val_env_set[v] = 1
                        successors = list(self.G.successors(v))
                        Val_successors = [self.Val_env[s] for s in successors]
                        Val_sys_successors = [self.Val_sys[s] for s in successors]
                        Val_sys_cur = self.Val_sys[v]
                        # write a line about potential states that are actually good with just there exists..
                        Val_sys_dec = [s for s in Val_sys_successors if (s < Val_sys_cur)]
                        Val_sys_dec_idx = [ii for ii in range(len(Val_sys_successors)) if (Val_sys_successors[ii] < Val_sys_cur)]
                        Val_env_dec = [Val_successors[ii] for ii in Val_sys_dec_idx]
                        if Val_sys_dec:
                            pre_win_set.append(v)
                            min_succ_Val_env_dec = Val_env_dec.index(min(Val_env_dec))
                            min_ii = Val_sys_dec_idx[min_succ_Val_env_dec]
                            min_successor = successors[min_ii]
                            # min_successor = successors[Val_successors.index(min(Val_successors))]
                            self.Val_env[v] = self.Val_env[min_successor]
                            pdb.set_trace()
        if ptype == 'e':
            for v in W0:
                if(self.Val_env_set[v] == 0):
                    self.Val_env_set[v] = 1
                    successors = list(self.G.successors(v))
                    if successors != []:
                        Val_successors = [self.Val_env[s] for s in successors]
                        max_successor = successors[Val_successors.index(max(Val_successors))]
                        self.Val_env[v] = self.Vweight[v] + self.Val_env[max_successor]
        return pre_win_set

    def determine_environment_value(self, W0, W0_succ, quant, ptype, win_agent):
        pre_win_set = []
        assert(win_agent == 'e')
        if ptype == 's':
            if (quant == 'forall'):
                for v in W0:
                    if(self.Val_env_set[v] == 0):
                        self.Val_env_set[v] = 1
                        successors = list(self.G.successors(v))
                        successors = [s for s in successors if (s in W0_succ)]
                        if successors!=[]:
                            pre_win_set.append(v)
                            Val_successors = [self.Val_env[s] for s in successors]
                            min_successor = successors[Val_successors.index(min(Val_successors))]
                            self.Val_env[v] = self.Val_env[min_successor]
            if (quant == 'exists'):
                for v in W0:
                    if(self.Val_env_set[v] == 0):
                        self.Val_env_set[v] = 1
                        valv = self.Val_sys[v]
                        successors = list(self.G.successors(v))
                        val1 = [self.Val_sys[s] for s in successors]
                        successors = [s for s in successors if (s in W0_succ)]
                        if successors!=[]:
                            pre_win_set.append(v)
                            Val_successors = [self.Val_env[s] for s in successors]
                            min_successor = successors[Val_successors.index(min(Val_successors))]
                            self.Val_env[v] = self.Val_env[min_successor]
        if ptype == 'e':
            for v in W0:
                if(self.Val_env_set[v] == 0):
                    self.Val_env_set[v] = 1
                    successors = list(self.G.successors(v))
                    if successors != []:
                        pre_win_set.append(v)
                        Val_successors = [self.Val_env[s] for s in successors]
                        max_successor = successors[Val_successors.index(max(Val_successors))]
                        self.Val_env[v] = self.Vweight[v] + self.Val_env[max_successor]
        return pre_win_set

    def determine_system_value(self, W0, quant, ptype, N, win_agent):
        assert(win_agent=='s')
        if(ptype  == 's'):
            for v in W0:
                if(self.Val_sys_set[v] == 0):
                    self.Val_sys_set[v] = 1
                    self.Val_sys[v] = N
        if(ptype == 'e'):
            if (quant == 'forall'):
                for v in W0:
                    if(self.Val_sys_set[v] == 0):
                        self.Val_sys_set[v] = 1
                        self.Val_sys[v] = N-1
            if (quant == 'exists'):
                for v in W0:
                    if(self.Val_sys_set[v] == 0):
                        self.Val_sys_set[v] = 1
                        self.Val_sys[v] = N-1
# Function to compute length of
# Function to return set of winning states as forms of vertices in V by taking in a list of lists: [[env_node, sys_nodes]] 
# of environment and system locations where env and sys are cell locations of the environment and system on the gridworld respectively
    def set_win_states(self, goal_list):
        goal_env = []
        goal_sys = []
        for g in goal_list:
            ne = g[0]
            ns = g[1]
            st = self.state(self.Ns, self.Ne, ns, ne)
            env_st = "v1_"+str(st)
            sys_st = "v2_"+str(st)
            goal_env.append(env_st)
            goal_sys.append(sys_st)
        Wgoal = [goal_env, goal_sys]
        return Wgoal

# Number of vertices with a value function:
    def get_val_numbers(self):
        num_val_phisys = 0
        num_val_phienv = 0
        for v in self.V:
            if self.Val_sys[v] != float('inf'):
                num_val_phisys += 1
            if self.Val_env[v] > 0:
                num_val_phienv += 1
        return num_val_phienv, num_val_phisys

# The following function is to synthesize the reachability winning set:
# + win_agent: the player ('s' or 'e') for which the winning set is being determined
# + goal: The set of states which is the goal for the winning agent. Goal is a list: [[goal_env], [goal_sys]] where goal_sys is the goal with system action states and goal_env is goal with environment action states
# + quant1: 'exists' or 'forall'; this is the quantifier for the other agent that is not the win_agent (env)
# + quant2: 'exists' or 'forall' this is the quantifier for the win_agent (sys)
# Outputs:
# + W: This is a winning set with two arguments: [W_env, W_sys]
# + Val: Value function for each element of the winning set. Converted to a dictionary from a list
    def win_reach(self, win_agent, goal, quant1, quant2):
        setting = 1
        setting2 = 2
        W = [[goal[0], goal[1]]]
        lW = len(W)
        W0 = W[lW-1]
        W0_sys = W0[1].copy()
        W0_env = W0[0].copy()
        if (win_agent == 's'):
            other_agent = 'e'
            quant_e = quant1
            quant_s = quant2
            for v in W0_sys:
                self.Val_sys[v] = 0
            for v in W0_env:
                self.Val_sys[v] = 0
        else:
            other_agent = 's'
            quant_s = quant1
            quant_e = quant2
            # assert(W0_sys == []) # We only want environment states in the original winning set of the system
            for v in W0_env:
                self.Val_env[v] = self.Vweight[v]
            W0_sys = []
            W = [[W0_env, W0_sys]] # Ignoring all the system states

        num_val_phienv, num_val_phisys = self.get_val_numbers()
        # pdb.set_trace() # Check to see if weights and values have been set well
        fixpoint=False
        N = 0
        while (not fixpoint):
            N += 1
            lW = len(W)
            if(lW>100):
                break
            if setting2 == 1:
                pre_sys = self.pre(W0_sys, 'e', quant_e)
                pre_env = self.pre(W0_env, 's', quant_s)
            if setting2 == 2:
                pre_sys = self.pre2(W0_sys, 'e', quant_e, win_agent)
                pre_env = self.pre2(W0_env, 's', quant_s, win_agent)

            W_ii = W[lW-1].copy()
            Wnew_sys = W_ii[1].copy()
            Wnew_env = W_ii[0].copy()

            if pre_env:
                Wnew_sys.extend(pre_env) # pre_env contains system states and is a predecessor to a environment winning set
                Wnew_sys = list(dict.fromkeys(Wnew_sys)) # Removes duplicates
            if pre_sys:
                Wnew_env.extend(pre_sys) # pre_sys contains environment states and is a predecessor to a system winning set
                Wnew_env = list(dict.fromkeys(Wnew_env)) # Removes duplicates
            if(win_agent=='s'):
                self.determine_system_value(pre_sys, quant_e, 'e', N, win_agent)
                self.determine_system_value(pre_env, quant_s, 's', N, win_agent)
            if(win_agent=='e' and setting == 1):
                pre_sys_win_set = self.determine_environment_value(pre_sys, W0_sys, quant_e, 'e', win_agent)
                pre_env_win_set = self.determine_environment_value(pre_env, W0_env, quant_s, 's', win_agent)
                Wnew_sys = W_ii[1].copy()
                Wnew_env = W_ii[0].copy()
                Wnew_sys.extend(pre_env_win_set)
                Wnew_env.extend(pre_sys_win_set)
                Wnew_sys = list(dict.fromkeys(Wnew_sys)) # Removes duplicates
                Wnew_env = list(dict.fromkeys(Wnew_env)) # Removes duplicates
            if(win_agent=='e' and setting == 2):
                pre_sys_win_set = self.determine_environment_value2(pre_sys, quant_e, 'e', win_agent)
                pre_env_win_set = self.determine_environment_value2(pre_env, quant_s, 's', win_agent)
                rem_sys = [st for st in pre_sys if st not in pre_sys_win_set] # Contains list of system vertices that need to be removed from system vertices
                rem_env = [st for st in pre_env if st not in pre_env_win_set] # Contains the list of env vertices that need to be removed from env vertices
                Wnew_sys = [st for st in Wnew_sys if st not in rem_env]
                Wnew_env = [st for st in Wnew_env if st not in rem_sys]

            num_val_phienv, num_val_phisys = self.get_val_numbers()
            Wcur = [Wnew_env, Wnew_sys]
            if(Wcur == W_ii):
                fixpoint = True
            W.append(Wcur)
            W0_sys = pre_env.copy()
            W0_env = pre_sys.copy()
        return W

    # Randomly choosing an initial condition 
    def choose_init(self, Wenv, Wsys):
        Wenv_pre_fixpoint1 = [s for s in Wenv[-1][0] if s in Wsys[-1][0]]
        Wenv_pre_fixpoint2 = [s for s in Wenv[-1][1] if s in Wsys[-1][1]]

        # Make sure both lists are non-empty:
        assert(Wenv_pre_fixpoint1!=[])
        assert(Wenv_pre_fixpoint2!=[])
        Val1 = [self.Val_env[s] for s in Wenv_pre_fixpoint1]
        Val2 = [self.Val_env[s] for s in Wenv_pre_fixpoint2]
        max_Val1 = max(Val1)
        max_Val2 = max(Val2)
        potential_init1 = [Wenv_pre_fixpoint1[ii] for ii in range(len(Wenv_pre_fixpoint1)) if Val1[ii] == max_Val1]
        potential_init2 = [Wenv_pre_fixpoint2[ii] for ii in range(len(Wenv_pre_fixpoint2)) if Val2[ii] == max_Val2]
        choose_max = "no"
        if choose_max == "yes":
            choice1_idx = random.choice(range(len(potential_init1)))
            choice2_idx = random.choice(range(len(potential_init2)))
            choices = [potential_init1[choice1_idx], potential_init2[choice2_idx]]
        else:
            choice1_idx = random.choice(range(len(Wenv_pre_fixpoint1)))
            choice2_idx = random.choice(range(len(Wenv_pre_fixpoint2)))
            choices = [Wenv_pre_fixpoint1[choice1_idx], Wenv_pre_fixpoint2[choice2_idx]]
        choice_idx = random.choice([0,1])
        return choices[0]

    # Write note for why we have Wsys here.
    # Just for asserting that the state in the winning set of the system
    def test_policy(self, q, Wsys):
        qsucc = list(self.G.successors(q))
        for qprime in qsucc:
            if (qprime not in Wsys[-1][1]):
                print("Err: Env actions not in system winning set")
                p, ns, ne = self.unpack_vertex(qprime)
                print("Env: "+str(ne)+" Sys: "+str(ns))
                assert(qprime in Wsys[-1][1])
        val_qsucc = [self.Val_env[qn] for qn in qsucc]
        max_val = max(val_qsucc)
        potential_qnext = [qsucc[ii] for ii in range(len(qsucc)) if val_qsucc[ii]==max_val]
        if len(potential_qnext) > 1:
            qnext = random.choice(potential_qnext)
        else:
            qnext = potential_qnext[0]
        return qnext

    def agent_policy(self, q, cones, Wsys):
        flag_win = False
        qsucc = list(self.G.successors(q))
        ntotal = len(qsucc)
        maximal_envwin_states = Wsys[-1][0]
        qsucc_win = [qn for qn in qsucc if (qn in maximal_envwin_states and qn not in cones)]
        ncorrect = len(qsucc_win)
        qsucc_win_val = [self.Val_sys[qn] for qn in qsucc_win]
        qval = self.Val_sys[q]
        qpotential = [qsucc_win[ii] for ii in range(len(qsucc_win)) if qsucc_win_val[ii] <= qval]
        qnext = random.choice(qpotential)
        # If you reached the goal, then stop there:
        if qnext in Wsys[0][0]:
            flag_win = True
        return qnext, flag_win, ntotal, ncorrect
