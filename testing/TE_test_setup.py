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
from coverage_propositions.lamda_functions import construct_lambda_functions
from map_representations.representation_conversions import abst_to_pixel, pixels_to_abstr
from map_representations.system_product import sys_to_product, sys_from_product
from map_representations.environment_product import env_to_product, env_from_product


# =============== Setting up abstraction / transition systems for the AVP system and the test environment ==================== #
# Function to load System and Environment transition dictionaries from pkl files
def load_transitions(fname_Tsys, fname_Tenv):
    with open(fname_Tsys, 'rb') as handle:
        Tsys = pickle.load(handle)

    with open(fname_Tenv, 'rb') as handle:
        Tenv = pickle.load(handle)

# ==================== Function to return general test run configuration: =================================== #
# test_run_config= trcg(Tsys, Tenv)
def construct_test_run_config(Tsys, Tenv):
    test_run_config = test_run_configuration(Tsys)
    return test_run_config

# ========================== Setting up propositions to decide the test case ==================================== #
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
        func = construct_lambda_functions(sys_prop_list[ii], env_prop_list[ii], connectives_list[ii])
        if func != []:
            lambda_functions.append(func)
        else:
            print("Err: Don't enter empty lists for system and environment propositions at the same time.")
    return lambda_functions

# Set weights for each proposition listed:
def set_weight_propositions(lambda_functions, weights):
    coverage_props_list = []
    assert(len(weights) == len(lambda_functions))
    for ii in range(len(weights)):
        coverage_props_list.append((lambda_functions[ii], weights[ii]))
    return coverage_props_list

# ======================= Setting up Test Configuration with static obstacles =============================== #
# For now we can only set static obstacles in the absence of an environment, and if there are no propositions that depend on the environment
def setup_static_obstacles(coverage_props_list):
    pass

# ================= Constructing the game graph based on the static obstacle test setup ===================== #
def construct_game_graph(test_config, Tsys, Tenv):
    pass

# ======================================== Synthesizing the test policy ======================================== #
def test_policy(system_state):
    pass