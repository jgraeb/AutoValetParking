import trio
import sys
sys.path.append('..') # enable importing modules from an upper directory:
# import 
from prepare.communication import *
from variables.global_vars import *
# import components
#from components.boxcomponent import BoxComponent
from components.game import Game
from components.simulation import Simulation
from components.map  import Map
from components.planner import Planner
from components.pedestrian import Pedestrian
from components.car import Car
from components.supervisor import Supervisor
from components.customer import Customer

async def main():
    global start_time
    start_time = trio.current_time()
    end_time = start_time + OPEN_TIME
    all_components = []
    print('--- Starting Parking Garage ---')
    async with trio.open_nursery() as nursery:
        map_sys = Map()
        all_components.append(map_sys)
        simulation = Simulation()
        all_components.append(simulation)
        supervisor = Supervisor(nursery = nursery)
        #all_components.append(supervisor)
        game = Game()
        all_components.append(game)
        planner = Planner(nursery=nursery)
        #all_components.append(planner)
        customer = Customer(average_arrival_rate = average_arrival_rate, average_park_time = average_park_time)
        #create communication channels
        set_up_channels(supervisor,planner, game, map_sys, customer, simulation)
        # start nursery
        for comp in all_components:
            nursery.start_soon(comp.run)
            await trio.sleep(0)
        nursery.start_soon(planner.run, game)
        nursery.start_soon(supervisor.run, planner)
        nursery.start_soon(customer.run,end_time,start_time, game)

trio.run(main)
