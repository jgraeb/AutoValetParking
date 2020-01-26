# Author: Josefine Graebener
# 
# Jan 24th 2020
#
# Specifications for an Automated Valet Parking System Supervisory Layer
# Environment includes failure cases: 1. Steering Problem, 2. Stop, 3. Lot Full
#
#  System simplified into 8 states:
# 'approaching' (X0)
# 'wait_dropoff' (X1)
# 'omw2park' (X2)
# 'parked' (P)
# 'omw2_pickup' (X4)
# 'wait_pickup' (X5)
# 'picked_up' (X6)
# 'rejected' (R)
# Transitiond defined below

from __future__ import print_function

import logging

from tulip import transys, spec, synth
# get package to export controller
from tulip import dumpsmach
# get package for FSM
from tulip.transys import transys as trs

logging.basicConfig(level=logging.WARNING)
logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
logging.getLogger('tulip.synth').setLevel(logging.WARNING)
logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)

# Create a finite transition system
sys = trs.FTS()

# Define the states of the system
sys.states.add_from(['X0', 'X1', 'X2', 'P', 'X4', 'X5', 'X6','R']) # Consider only parking state at current stage
sys.states.initial.add('X0')    # start in state X0

# need to add labels to the transitions
#sys.actions |= ['drive', 'stay']
#sys.actions.add('drive')
#sys.actions.add('stay')
# Define the allowable transitions
sys.transitions.add_comb({'X0'}, {'X0', 'X1'})
sys.transitions.add_comb({'X1'}, {'X1', 'X2', 'R'})
sys.transitions.add_comb({'X2'}, {'X2', 'P'})
sys.transitions.add_comb({'P'}, {'P', 'X4'})
sys.transitions.add_comb({'X4'}, {'X4', 'X5'})
sys.transitions.add_comb({'X5'}, {'X5', 'X6'})
sys.transitions.add_comb({'X6'}, {'X6'})
sys.transitions.add('R', 'R')

# Add atomic propositions to the states
sys.atomic_propositions.add_from({'approaching','wait_dropoff','omw2park','parked','omw2_pickup','wait_pickup','picked_up','rejected'})
sys.states.add('X0', ap={'approaching'})
sys.states.add('X1', ap={'wait_dropoff'})
sys.states.add('X2', ap={'omw2park'})
sys.states.add('P', ap={'parked'})
sys.states.add('X4', ap={'omw2_pickup'})
sys.states.add('X5', ap={'wait_pickup'})
sys.states.add('X6', ap={'picked_up'})
sys.states.add('R', ap={'rejected'})


#set([e[1] for e in sys.transitions.find('X0', with_attr_dict={'sys_action':'jump'})])
# save the transition system as a pdf
sys.save('FSM_sup_sys.pdf')
# if IPython and Matplotlib available
#sys.plot() 

# @environ_section@
env_vars = set() # empty set
env_init = set() # empty set
env_prog = set() # empty set
env_safe = set() # empty set
# General
env_vars |= {'requested'} # if the car has been requestested
# Exceptions - Failure cases
# Car
env_vars |= {'exception_steering'} # Steering system problem
env_vars |= {'exception_stop'} # Car has stopped (needs to be rerouted - planning layer TBD)
#env_vars |= {'exception_spot_blocked'} # ADDblocked spot for replanning
#env_vars |= {'done'}
# Parking  spots
env_vars |= {'no_spot'} # Parking lot is full - no spots left
# Initial env vars
env_init |= {'!no_spot'} # at the beginning there are spots available
# Progess
env_prog |= {'requested'} # the car will always eventually be requested
env_prog |= {'requested && !exception_stop'} # the car will always eventually be requested and not stopped
env_prog |= {'!exception_stop'} # always eventually the car will not be stopped
env_prog |= {'!no_spot'} # always eventually there will be a spot
#env_prog |= {'!exception_stop && !exception_spot_blocked'}

# System specification
# @specs_setup_section@
sys_vars = set()
sys_init = set()
sys_prog = set()
sys_safe = set()
# Augment the system description to make it GR(1)
# @ set system variables
sys_vars |= {'random_spot'}     # pick an available spot randomly     
sys_vars |= {'easy_spot'}       # pick a spot for a car, which is easier to reach (for steering problem)
# sys_vars |= {'new_spot'}      # to be added for replanning
# @ set initial values
sys_init = {'!random_spot'}     # beginning no spot picked
sys_init = {'!easy_spot'}       # beginning no spot picked
# @ set progress
sys_prog |= {'(picked_up || rejected)'}             # []<> car will leave (picked up or rejected)
# @  set safety
sys_safe |= {'((approaching && exception_stop) -> X(approaching))'}
sys_safe |= {'((approaching && !exception_stop) -> X(wait_dropoff))'}
sys_safe |= {'((wait_dropoff && !exception_stop && no_spot) -> X(rejected))'}
sys_safe |= {'((wait_dropoff && !exception_stop && !exception_steering  && !no_spot) -> X(omw2park && random_spot))'}
sys_safe |= {'((wait_dropoff && !exception_stop && exception_steering &&  !no_spot) -> X(omw2park && easy_spot))'}
sys_safe |= {'((wait_dropoff && exception_stop) -> X(wait_dropoff))'}
sys_safe |= {'((omw2park && exception_stop) -> X(omw2park))'} 
sys_safe |= {'((omw2park && !exception_stop) -> X(parked))'} 
#sys_safe |= {'((omw2park && exception_spot_blocked) -> X(omw2park))'}  # + newspot (ADD replanning)
sys_safe |= {'((parked && (!requested || exception_stop) ) -> X(parked))'}
sys_safe |= {'((parked && requested && !exception_stop) -> X(omw2_pickup))'}
sys_safe |= {'((omw2_pickup && !exception_stop) -> X(wait_pickup))'}
sys_safe |= {'((omw2_pickup && exception_stop) -> X(omw2_pickup))'}
sys_safe |= {'((wait_pickup && !exception_stop ) -> X(picked_up))'}
sys_safe |= {'((wait_pickup && exception_stop ) -> X(wait_pickup))'}
sys_safe |= {'picked_up -> X(picked_up)'}
sys_safe |= {'rejected -> X(rejected)'}


# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

print(specs.pretty())
specs.moore = True
# synthesizer should find initial system values that satisfy
# `env_init /\ sys_init` and work, for every environment variable
# initial values that satisfy `env_init`.
specs.qinit = '\E \A'
ctrl = synth.synthesize(specs, sys=sys)
assert ctrl is not None, 'unrealizable'
# @synthesize_end@

#
# Generate a graphical representation of the controller for viewing,
# or a textual representation if pydot is missing.
#
# @plot_print@
if not ctrl.save('sup_ctrl.png'):    
    print(ctrl)
# @plot_print_end@

# appended as in documentation chapter 6
dumpsmach.write_python_case("sup_ctrl.py", ctrl, classname="ExampleCtrl")