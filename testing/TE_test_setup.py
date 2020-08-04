# Test run for the AVP system
import numpy as np
import random
from random import randrange
import importlib
import itertools
import pickle
import os
import math
import networkx as nx
import pdb

from dynamic_environment.gridworld_classes.gridworld_class import GridWorld 
from dynamic_environment.gridworld_classes.Player_class import Player
from dynamic_environment.transitions_classes.General_Game_Graph_class import GeneralGameGraph
from static_environment.transitions.test_run_configuration_transitions import test_run_configuration 

# Setting up abstraction / transition systems for the AVP system and the test environment
# Function to load System and Environment transition dictionaries from pkl files
def load_transitions(fname_Tsys, fname_Tenv):
    with open(fname_Tsys, 'rb') as handle:
        Tsys = pickle.load(handle)

    with open(fname_Tenv, 'rb') as handle:
        Tenv = pickle.load(handle)

# Function to return general test run configuration:
# test_run_config= trcg(Tsys, Tenv)
def construct_test_run_config(Tsys, Tenv):
    test_run_config = test_run_configuration(Tsys)
    return test_run_config

# Setting up propositions to decide the test case

# Setting up Test Configuration with static obstacles

# Constructing the game graph based on the static obstacle test setup

# Synthesizing the test run:
