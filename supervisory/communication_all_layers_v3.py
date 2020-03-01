import curses
from ipdb import set_trace as st
import numpy as np
import trio
import random

average_arrival_rate = 0.5 # per second
beta = 1/average_arrival_rate
average_park_time = 10 # seconds
MAX_BUFFER_SIZE = np.inf


def get_current_time():
    return trio.current_time() - start_time

def create_unidirectional_channel(sender, receiver, max_buffer_size, name = False):
    out_channel, in_channel = trio.open_memory_channel(max_buffer_size)
    if name:
        sender.out_channels[name] = out_channel
        receiver.in_channels[name] = in_channel
    else:
        sender.out_channels[receiver.name] = out_channel
        receiver.in_channels[sender.name] = in_channel

def create_bidirectional_channel(compA, compB, max_buffer_size):
    create_unidirectional_channel(sender=compA, receiver=compB, max_buffer_size=max_buffer_size)
    create_unidirectional_channel(sender=compB, receiver=compA, max_buffer_size=max_buffer_size)

class CarInfo:
    def __init__(self, arrive_time, depart_time):
        self.arrive_time = arrive_time
        self.depart_time = depart_time

class BoxComponent:
    def __init__(self):
        self.in_channels = dict()
        self.out_channels = dict()

#    def _initialize_channels(self, num_channels):
#        channels = []
#        for _ in range(num_channels):
#            channels.append(None)
#        return channels
#
#    def _find_open_channel(self, channel_list):
#        for idx in range(len(channel_list)):
#            if channel_list[idx] == None:
#                return idx
#        # all channels are closed!
#        return -1

class Map(BoxComponent): # not yet used
    def __init__(self,nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.positions = dict()

    def send_position(self,car):
        print('sending car position')
        pos = [value  for (key, value) in self.positions() if key == car.name]
        self.out_channels['Planner'].send(car,pos)

    def receive_request(self):
        print('Receiving request')
        car = self.in_channels['Planner'].receive()
        self.send_position(car)

    async def run(self):
        async for car in self.in_channels['Planner']:
            await self.send_position(car)


class Planner(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.nursery = nursery
        self.cars = []

    async def send_directive_to_car(self, car, ref):
        print('Planner sending Directive to {0}'.format(car.name))
        await self.out_channels[car.name].send(ref)
        await trio.sleep(1)

    async def update_car_response(self, receive_response_channel):
        async with receive_response_channel:
            async for response in receive_response_channel:
                car = response[0]
                resp = response[1]
                print('Planner receiving "{0}" response from {1}'.format(resp,car.name))
                await trio.sleep(0)
                await self.send_response_to_supervisor(car,resp)

    async def send_response_to_supervisor(self,car,response):
        print('Planner sending "{0} - {1}" response to Supervisor'.format(response,car.name))
        await self.out_channels['Supervisor'].send((car,response))
        await trio.sleep(0)
    
    async def check_supervisor(self,send_response_channel):
        async for directive in self.in_channels['Supervisor']:
            car = directive[0]
            todo = directive[1]
            if todo == 'Park':
                print('Planner receiving directive from Supervisor to park {0} in Lot tbd'.format(car.name))
                self.cars.append(car)
                create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                self.nursery.start_soon(car.run,send_response_channel)
                ref =[1,1]
            elif todo == 'Pickup':
                print('Planner receiving directive from Supervisor to retrieve {0}'.format(car.name))
                ref = [0,0]
            await self.send_directive_to_car(car, ref)

    async def run(self):
        send_response_channel, receive_response_channel = trio.open_memory_channel(25)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.check_supervisor,send_response_channel)
            await trio.sleep(1)
            nursery.start_soon(self.update_car_response,receive_response_channel)


class Car(BoxComponent):
    def __init__(self, arrive_time, depart_time):
        super().__init__()
        self.name = 'Car {}'.format(id(self))
        self.arrive_time = arrive_time
        self.depart_time = depart_time
        self.ref = None

    async def update_planner_command(self):
        async for directive in self.in_channels['Planner']:
            self.ref = directive
            print('{0} - Receiving Directive from Planner'.format(self.name))
            #self.ref = await self.in_channels['Planner'].receive()
            #await trio.sleep(0)
            print('{0} - Tracking reference... {1}'.format(self.name,self.ref))
            #self.track_reference

    async def track_reference(self):
        #self.ref = await self.in_channels['Planner'].receive()
        print('{0} - Tracking reference... {1}'.format(self.name,self.ref))
        await trio.sleep(1)

    async def send_response(self,send_response_channel):
        await trio.sleep(1)
        response = 'Completed'
        #await self.out_channels['Planner'].send(response)
        print('{} sending response to Planner'.format(self.name))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def run(self,send_response_channel):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_planner_command)
            await trio.sleep(1)
            #nursery.start_soon(self.track_reference)
            nursery.start_soon(self.send_response,send_response_channel)


class Supervisor(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        #self.nursery = nursery

    async def send_directive_to_planner(self, car,ref):
        directive = [car,ref]
        await self.out_channels['Planner'].send(directive)

    async def update_planner_response(self):
        response = await self.in_channels['Planner'].receive()
        car = response[0]
        resp = response[1]
        print('{0} - Receiving "{0} - {1}" Response from Planner'.format(car.name,resp))
        await trio.sleep(0)

    async def process_queue(self):
        #ref = [[0,0],[1,1]]
        accept_condition = True
        async for car in self.in_channels['Customer']:
            if accept_condition:
                print('{} has been accepted!'.format(car.name))
                #await trio.sleep(0)
                await self.send_directive_to_planner(car,'Park')
            else:
                print('A car has been rejected!')

    async def request_queue(self):
        async for car in self.in_channels['Request']:
            await self.send_directive_to_planner(car, 'Pickup')

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.process_queue)
            nursery.start_soon(self.request_queue)
            nursery.start_soon(self.update_planner_response)

class Customer(BoxComponent):
    def __init__(self, average_arrival_rate, average_park_time):
        super().__init__()
        self.name = self.__class__.__name__
        self.average_arrival_rate = average_arrival_rate
        self.average_park_time = average_park_time
        self.cars = []

    def generate_car(self):
        arrive_time = get_current_time()
        depart_time = arrive_time + np.random.exponential(self.average_park_time)
        car = Car(arrive_time=arrive_time, depart_time=depart_time)
        return car
        
    async def run(self):
        while True:
            # spawns cars according to exponential distribution
            await trio.sleep(np.random.exponential(1/self.average_arrival_rate))
            car = self.generate_car()
            self.cars.append(car)
            print("Car with ID {0} arrives at {1:.3f}".format(car.name, car.arrive_time))
            await self.out_channels['Supervisor'].send(car)
            now = get_current_time()
            for i,car in enumerate(self.cars): # checks for requested cars
                if car.depart_time <= now:
                    print('{0} is requested at {1:.3f}'.format(car.name,car.depart_time))
                    await self.out_channels['Request'].send(car)
                    self.cars.pop(i)
            # Random sleeps help trigger the problem more reliably
            await trio.sleep(random.random())

async def main():
    global start_time
    start_time = trio.current_time()
    all_components = []
    async with trio.open_nursery() as nursery:
        supervisor = Supervisor()#(nursery=nursery)
        all_components.append(supervisor)
        planner = Planner(nursery=nursery)
        all_components.append(planner)
        customer = Customer(average_arrival_rate = average_arrival_rate, average_park_time = average_park_time)
        all_components.append(customer)

        create_bidirectional_channel(supervisor,planner,max_buffer_size=np.inf)
        create_unidirectional_channel(sender=customer, receiver=supervisor, max_buffer_size=np.inf)
        create_unidirectional_channel(sender=customer, receiver=supervisor, max_buffer_size=np.inf, name='Request')

        for comp in all_components:
            nursery.start_soon(comp.run)

trio.run(main)
