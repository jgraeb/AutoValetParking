# Automated Valet Parking - Environment Customer Component
# Josefine Graebener
# California Institute of Technology
# March, 2020

from components.boxcomponent import BoxComponent
import trio
import random
from variables.global_vars import *
from prepare.communication import *
from components.car import Car
from ipdb import set_trace as st

class Customer(BoxComponent):
    def __init__(self, average_arrival_rate, average_park_time):
        super().__init__()
        self.name = self.__class__.__name__
        self.average_arrival_rate = average_arrival_rate
        self.average_park_time = average_park_time
        self.cars = []

    def generate_car(self,start_time):
        arrive_time = get_current_time(start_time)
        depart_time = arrive_time + np.random.exponential(self.average_park_time)
        car = Car(arrive_time=arrive_time, depart_time=depart_time)
        return car
        
    async def run(self,end_time,start_time, gme):
        now = trio.current_time()
        #garage_open = True
        while True:
            # check if dropoff spot is free
            # while not gme.dropoff_free():
            #     await trio.sleep(5)
            # spawns cars according to exponential distribution
            await trio.sleep(np.random.exponential(1/self.average_arrival_rate))
            if gme.dropoff_free():
                car = self.generate_car(start_time)
                #self.cars.append(car)
                car.x = START_X
                car.y = START_Y
                car.yaw = START_YAW
                print("Car with ID {0} arrives at {1:.3f}".format(car.name, car.arrive_time))
                await self.out_channels['Supervisor'].send(car)
                self.cars.append(car)
                accept = await self.in_channels['Supervisor'].receive() # checks if car is accepted by the garage
                if (accept == False):
                    self.cars.pop()
                await trio.sleep(5)
            now = get_current_time(start_time)
            for i,cars in enumerate(self.cars): # checks for requested cars
                if cars.depart_time <= now and cars.parked:
                    print('{0} is requested at {1:.3f}'.format(cars.name,now))
                    cars.depart_time = now
                    cars.requested = True
                    await self.out_channels['Request'].send(cars)
                    self.cars.pop(i)
            # self.cars.append(car)
            #print(self.cars)
            #if (now>=end_time):
            #    garage_open = False
            # await trio.sleep(5) # give car time to start tracking
            # Random sleeps help trigger the problem more reliably
            await trio.sleep(random.random())
        print('--------- It is {} time  --------'.format(trio.current_time()))
        print('The garage is closed now!')
            