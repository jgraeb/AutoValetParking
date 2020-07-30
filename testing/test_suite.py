# Automated Valet Parking - Test Suite - Main Testing Component (for T&E of car component)
# Josefine Graebener
# California Institute of Technology
# July, 2020

# import variables
from variables.geometries import UPPER_SPOTS, LOWER_SPOTS, MIDDLE_BOX
from variables.global_vars import MAX_NO_PARKING_SPOTS, TOW_TIME, DELAY_THRESH, start_walk_lane, end_walk_lane, start_walk_lane_2, end_walk_lane_2, SCALE_FACTOR_PLAN as SFP, START_X, START_Y, START_YAW
from variables.parking_data import test_spots as parking_spots
# import communication functions
from prepare.communication import get_current_time
# import packages
import trio
import random
from ipdb import set_trace as st
# import components
from prepare.boxcomponent import BoxComponent
from components.car import Car
# from testing.test_pedestrian import Pedestrian

class TestSuite(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.nursery = nursery
        self.counter = 1
        self.cars = []
        self.spot_no = len(list(parking_spots.keys()))
        self.testpedlist = dict()
        self.parking_spots = dict([(i, ("Vacant", "None")) for i in list(parking_spots.keys())])
        self.Logger = None

    async def generate_car(self, start_time, park_time):
        print('TEST SUITE - Generating car')
        arrive_time = get_current_time(start_time)
        depart_time = arrive_time + park_time
        car = Car(arrive_time=arrive_time, depart_time=depart_time)
        car.id = self.counter
        self.counter=self.counter+1
        car.x = START_X
        car.y = START_Y
        car.yaw = START_YAW
        # update game
        await self.out_channels['Game'].send(car)
        print("Car with Name {0} and ID {1} arrives at {2:.3f}".format(car.name,car.id, car.arrive_time))
        await self.out_channels['Planner'].send([car, ('Park', self.spot_no)])
        self.cars.append(car)

    async def send_car_to_spot(self,car,spot):
        await self.out_channels['Planner'].send([car, ('Park', spot)])

    async def request_car(self,car):
        now = trio.current_time()
        print('{0} is requested at {1:.3f}'.format(car.name,now))
        car.depart_time = now
        car.requested = True
        await self.out_channels['Planner'].send([car, 'Pickup'])

    async def start_ped(self,TestPed):
        # spawn the ped in loc A and save to ped list
        print('Spawning ped')

    async def stop_ped(self,TestPed):
        print('Stop ped')

    async def ped_walk_west(self,TestPed):
        print('Ped walking west')

    async def ped_walk_east(self,TestPed):
        print('Ped walking east')

    def get_next_env_action(self, response):#, TestPed, TestSupervisor, TestObstacles):
        print('Read test script here')
        # read the script
        # call the actions from above here depending on sys state

    async def update_sys_state(self, Planner):
        print('TEST SUITE - updating system state')
        async with self.in_channels['Planner']:
            async for response in self.in_channels['Planner']:
                print('Response received')
                self.get_next_env_action(response)

    async def run_test(self,Planner):
        print('Testing')
        await self.update_sys_state(Planner)

    async def run(self,Planner): #TestPed, TestSupervisor, TestObstacles
        print('TEST SUITE - started')
        now = trio.current_time()
        park_time = 300 #
        await self.generate_car(now, park_time)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.run_test,Planner)


        

