# Global variables for T&E generation
import sys
sys.path.append('..')

# Variables for system states:
Nsys1 = 50 # Adjust this depending on how many total physical locations are possible for a car
Ncars = 1 # Number of cars in simulation
Nsys2 = 4 # Number of possible statuses each car can take 
NSYS = (Nsys1*Nsys2)^Ncars # Total number of system states in the product automaton

# Variables for environment states:
Nped = 5 # Number of states the pedestrian can occupy
Nsup = 9 # Number of states the supervisor can occupy
Nobs = 1 # Number of "static" obstacles on test grid. These are obstacles that either "appear" (1) or "disappear" (0) and are separate
# from those that are initially configured on the grid
Nobs_states= 2
NENV = Nped*Nsup*Nobs_states^Nobs # Total number of environment states in the product automaton