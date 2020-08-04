# ======================= Overview ========================= #
# % ------------------------------ Class Player ----------------------------% #
# This class defines a player, its transitions of the gridworld and its policies
# + succ: Function that returns the set of successors to a player's node
# + pre: Function that returns set of predecessors to a player's node. Input node is in the form of a node id
# + get_transitions: Returns players physical transition on the grid
import numpy as np
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

import gridworld_class as gc
import Game_graph_class as gg

class Player:
    def __init__(self, name, grid, player_transitions_type, ptype):
        self.name = name
        self.type = ptype
        self.grid = grid
        self.transitions = None
        self.set_transitions(player_transitions_type)
        self.nstates = self.get_nstates()
        self.get_cur_state = None
        
    def set_transitions(self, player_transitions_type):
        gridT = self.grid.gridT
        C2N = self.grid.cell2node
        if (player_transitions_type[0]=='all'):
            self.transitions = gridT
        if(player_transitions_type[0]=='specific'):
            col = player_transitions_type[1]
            if(len(player_transitions_type) == 2):
                bump_idx = []
            else:
                bump_idx = player_transitions_type[2]
            
            T = [[] for ii in range(self.grid.Nrows * self.grid.Ncols)]
            for row in range(1, self.grid.Nrows+1):
                if(row==1):
                    cell = [row, col]
                    cell_trans = [[row+1, col]]
                elif(row == self.grid.Nrows):
                    cell = [row, col]
                    cell_trans = [[row-1, col]]
                else:
                    cell = [row, col]
                    cell_trans = [[row+1, col], [row-1, col]]
                
                T[C2N[(cell[0], cell[1])] - 1] = [C2N[(c[0], c[1])] for c in cell_trans]
                
                if(bump_idx):
                    # Clearing the transition in the cell of the bumped row and adding transitions to the one adjacent to it
                    if(row == bump_idx):
                        cell = [row, col]
                        T[C2N[(cell[0], cell[1])] - 1] = []
                        
                        cell = [row, col+1]
                        cell_trans = [[row+1, col+1], [row-1, col+1]]
                        T[C2N[(cell[0], cell[1])] - 1] = [C2N[(c[0], c[1])] for c in cell_trans]
                    if (row == (bump_idx - 1)):
                        cell1 = [row, col]
                        cell1_trans = [[row-1, col], [row, col+1]]
                        cell2 = [row, col+1]
                        cell2_trans = [[row, col], [row+1, col+1]]
                        T[C2N[(cell1[0], cell1[1])] - 1] = [C2N[(c[0], c[1])] for c in cell1_trans]
                        T[C2N[(cell2[0], cell2[1])] - 1] = [C2N[(c[0], c[1])] for c in cell2_trans]
                    if(row == bump_idx + 1):
                        cell1 = [row, col]
                        cell1_trans = [[row+1, col], [row, col+1]]
                        cell2 = [row, col+1]
                        cell2_trans = [[row, col], [row-1, col+1]]
                        T[C2N[(cell1[0], cell1[1])] - 1] = [C2N[(c[0], c[1])] for c in cell1_trans]
                        T[C2N[(cell2[0], cell2[1])] - 1] = [C2N[(c[0], c[1])] for c in cell2_trans]     
            self.transitions = T.copy()
        
        # Directly inputing transitions from nodes to nodes
        if(player_transitions_type[0]=='very_specific'):
            T = [[] for ii in range(self.grid.Nrows * self.grid.Ncols)]
            transitions = player_transitions_type[1]
            for k, val in transitions.items():
                T[k-1] = val
            self.transitions = T.copy()
        
    def get_transitions(self):
        return self.transitions

    def get_type(self):
        return self.type

    def get_nstates(self):
        N = 0
        for t in self.transitions:
            if t:
                N = N+1
        return N

    def succ(self, node):
        S = []
        return S

    def pre(self, node):
        P = []
        return P
    
