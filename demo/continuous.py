# Automated Valet Parking - Continuous Simulation
# Josefine Graebener
# California Institute of Technology
# March, 2020

import trio
import sys
sys.path.append('..') # enable importing modules from an upper directory:
# import
from prepare.communication import set_up_channels
from variables.global_vars import average_arrival_rate, average_park_time, OPEN_TIME
# import components
from prepare.boxcomponent import Time
from components.game import Game
from animation.simulation import Simulation
from components.map  import Map
from components.planner import Planner
from environment.pedestrian import Pedestrian
from components.car import Car
from components.supervisor import Supervisor
from environment.customer import Customer
from components.tow_truck import TowTruck
from testing.static_obstacle import Obstacles

import logging
import logging.config
logging.config.fileConfig('../prepare/Logging/logging.conf', disable_existing_loggers=False, defaults={'logfilename': 'logs/AVP_sim.log'})
# create logger
logger = logging.getLogger('AVP')

# set to normal mode
TESTING_MODE = False

async def main():
    #global START_TIME
    START_TIME = trio.current_time()
    END_TIME = START_TIME + OPEN_TIME
    time_sys = Time(START_TIME, END_TIME)
    all_components = []
    print('--- Starting Parking Garage at time: '+str(START_TIME))
    async with trio.open_nursery() as nursery:
        obstacles = Obstacles()
        map_sys = Map()
        all_components.append(map_sys)
        simulation = Simulation()
        all_components.append(simulation)
        supervisor = Supervisor(nursery = nursery)
        #all_components.append(supervisor)
        game = Game()
        #all_components.append(game)
        planner = Planner(nursery=nursery)
        #all_components.append(planner)
        customer = Customer(average_arrival_rate = average_arrival_rate, average_park_time = average_park_time)
        #create communication channels
        tow_truck = TowTruck()
        all_components.append(tow_truck)
        set_up_channels(supervisor,planner, game, map_sys, customer, simulation,tow_truck)
        # start nursery
        for comp in all_components:
            nursery.start_soon(comp.run)
            await trio.sleep(0)
        nursery.start_soon(game.run,logger)
        nursery.start_soon(planner.run, game, time_sys, logger, obstacles,simulation)
        nursery.start_soon(supervisor.run, planner, time_sys, simulation,logger)
        nursery.start_soon(customer.run,END_TIME,START_TIME, game)

trio.run(main)
