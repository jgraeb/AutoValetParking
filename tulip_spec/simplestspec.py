# Author: Josefine Graebener (jgraeb) and Jiaqi Yan 
# 
# Oct 6th 2019
#
# Specifications for a Automated Valet Parking System 
# Environment is the if the path is clear (!clear or clear) and if the car is requested (!request or request)
#
#  System simplified into 3 states:
#  At Drop-Off
#  Parked
#  At Pick-Up

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
sys.states.add_from(['X0', 'X1', 'X2', 'X3', 'X4', 'X5', 'X6', 'X7', 'X8', 'X9', 'X10', 'X11', 'P1']) # Consider only 1 parking lot at current stage
sys.states.initial.add('X0')    # start in state X0

#sys.actions |= ['drive', 'stay']
# Define the allowable transitions
sys.transitions.add_comb({'X0'}, {'X0', 'X1'})
#sys.transitions.add_comb({'X1'}, {'X1', 'X2', 'P1'})
sys.transitions.add_comb({'X1'}, {'X1', 'P1'})
#sys.transitions.add_comb({'P1'}, {'P1', 'X2'})
sys.transitions.add_comb({'P1'}, {'P1', 'X7'})
sys.transitions.add_comb({'X7'}, {'X7', 'X9'})
sys.transitions.add_comb({'X9'}, {'X9', 'X11'})
# sys.transitions.add_comb({'X2'}, {'X2', 'X3'})
# sys.transitions.add_comb({'X3'}, {'X3', 'X4'})
# sys.transitions.add_comb({'X4'}, {'X4', 'X5'})
# sys.transitions.add_comb({'X5'}, {'X5', 'X6'})
# sys.transitions.add_comb({'X6'}, {'X6', 'X7'})
# sys.transitions.add_comb({'X7'}, {'X7', 'X8'})
# sys.transitions.add_comb({'X8'}, {'X8', 'X9'})
# sys.transitions.add_comb({'X9'}, {'X9', 'X10'})
# sys.transitions.add_comb({'X10'}, {'X10', 'X11'})
sys.transitions.add('X11', 'X11')
# need to add labels to the transitions

# Add atomic propositions to the states
sys.atomic_propositions.add_from({'at_drop_off','at_entrance','driving_to_leave','at_pick_up','at_P1','leave'})
sys.states.add('X0', ap={'at_drop_off'})
sys.states.add('X1', ap={'at_entrance'})
sys.states.add('X7', ap={'driving_to_leave'})
sys.states.add('X9', ap={'at_pick_up'})
sys.states.add('X11', ap={'leave'})
sys.states.add('P1', ap={'at_P1'})

#set([e[1] for e in sys.transitions.find('X0', with_attr_dict={'sys_action':'jump'})])
# save the transition system as a pdf
sys.save('system_FiniteStateMachine.pdf')
# if IPython and Matplotlib available
#sys.plot() 

# @environ_section@
env_vars = {'clear'}     # if the path is clear
env_vars |= {'requested'} # if the car has been requestested
# Park implies eventually requested: [](at_P1 -> <>requested)
# Requested implied eventually picked up: [](requested -> <>picked_up)
env_init = set()              # empty set
env_prog = {'clear'} # the path will always eventually be clear
env_prog |= {'clear && requested'} # the car will always eventually be requested and the path will be clear
env_safe = set()

# System specification

# @specs_setup_section@
# Augment the system description to make it GR(1)
sys_vars = set()        # infer the rest from TS
sys_init = set()        
sys_prog = {'leave'}             # []<> leave
sys_safe = {'((at_drop_off && !clear) -> X(at_drop_off))'}
sys_safe |= {'((at_drop_off && clear) -> X(at_entrance))'}
sys_safe |= {'((at_entrance && !clear) -> X(at_entrance))'}
sys_safe |= {'((at_entrance && clear) -> X(at_P1))'} 
sys_safe |= {'((at_P1 && (!requested || !clear)) -> X(at_P1))'}
sys_safe |= {'((at_P1 && requested && clear) -> X(driving_to_leave))'}
sys_safe |= {'((driving_to_leave && !clear) -> X(driving_to_leave))'}
sys_safe |= {'((driving_to_leave && clear) -> X(at_pick_up))'}
sys_safe |= {'((at_pick_up && !clear) -> X(at_pick_up))'}
sys_safe |= {'((at_pick_up && clear) -> X(leave))'}


# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

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
#if not ctrl.save('simplestspec_ctrl.png'):
#    print(ctrl)
# @plot_print_end@

# appended as in documentation chapter 6
dumpsmach.write_python_case("simplestspec_ctrl.py", ctrl, classname="ExampleCtrl")