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
from coverage_propositions.lamda_functions import construct_lambda_function

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
# Return propositions you want to cover in a single test run
# Format: sys_prop_list = [[2,3,4], [6,7,8]], env_prop_list = [[3,6,7], [8,9]], connectives = ["and", "or", "and"]
# Ex: Lambda function returned for sys_prop_list[0] and env_prop_list[0] would be: lambda ns, ne: ns in sys_prop_lsit[0] and ne in env_prop_list[0]
# If only system states are pertinent, leave the env_prop_list at the corresponding index empty
def setup_propositions(sys_prop_list, env_prop_list, connectives_list):
    Nprop = len(connectives_list)
    assert(Nprop == len(sys_prop_list))
    assert(len(sys_prop_list) == Nprop)
    lambda_functions = []
    for ii in range(Nprop):
        func = construct_lambda_function(sys_prop_list[ii], env_prop_list[ii], connectives_list[ii])
        if func != []:
            lambda_functions.append(func)
        else:
            print("Err: Don't enter empty lists for system and environment propositions at the same time.")
    return lambda_functions

# Setting up Test Configuration with static obstacles

# Constructing the game graph based on the static obstacle test setup

# Synthesizing the test run:
