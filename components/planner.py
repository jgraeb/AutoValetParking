from components.boxcomponent import BoxComponent
import trio
import _pickle as pickle
import motionplanning.end_planner as path_planner
from prepare.communication import *
from variables.global_vars import *
from variables.parking_data import parking_spots
#from motionplanning.parking_data import parking_spots

class Planner(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.nursery = nursery
        self.cars = []
        self.parked = dict()
        self.assigned = dict()

    async def get_car_position(self,car):
        print('Planner - Sending position request to Map system')
        await self.out_channels['Map'].send(car)
        car, pos = await self.in_channels['Map'].receive()
        #if car.name in self.parked:
        #    pos = parking_spots[spot]
        # if self.parked.get(car.name, False):
        #     pos = parking_spots[self.parked.get(car.name)]
        #     start = np.zeros(4)
        #     start[0] = pos[0]*SCALE_FACTOR_PLAN
        #     start[1] = pos[1]*SCALE_FACTOR_PLAN
        #     start[2] = np.deg2rad(pos[2])
        #     print(start)
        #     return start
        #print(pos)
        return pos

    async def find_spot_coordinates(self, spot):
        return parking_spots[spot]

    async def send_directive_to_car(self, car, end):
        if self.parked.get(car.name, False):
            start = parking_spots[self.parked.get(car.name)]
            print(start)
            start[2] = start[2]
            print(start)
        else:
            pos = await self.get_car_position(car)
            start = np.zeros(4)
            start[0] = pos[0]/SCALE_FACTOR_PLAN
            start[1] = pos[1]/SCALE_FACTOR_PLAN
            start[2] = -np.rad2deg(pos[2])
        # get trajectory from Planning Graph
        print(start)
        traj = await self.get_path(start,end) 
        print('Planner - sending Directive to {0}'.format(car.name))
        await self.out_channels[car.name].send(traj)
        await trio.sleep(1)

    async def get_path(self, start, end): 
        sys.path.append('../motionplanning')
        with open('planning_graph_refined.pkl', 'rb') as f:
            planning_graph = pickle.load(f)
        traj = path_planner.get_mpc_path(start,end,planning_graph)
        return traj

    async def update_car_response(self, receive_response_channel):
        async with receive_response_channel:
            async for response in receive_response_channel:
                car = response[0]
                resp = response[1]
                print('Planner - receiving "{0}" response from {1}'.format(resp,car.name))
                if response[1]=='Parked':
                    self.parked.update({car.name: (self.assigned.get(car.name))})
                    print(self.parked)
                await trio.sleep(0)
                await self.send_response_to_supervisor(car,resp)

    async def send_response_to_supervisor(self,car,response):
        print('Planner - sending "{0} - {1}" response to Supervisor'.format(response,car.name))
        await self.out_channels['Supervisor'].send((car,response))
        await trio.sleep(0)
    
    async def check_supervisor(self,send_response_channel,Game):
        async for directive in self.in_channels['Supervisor']:
            car = directive[0]
            todo = directive[1]
            if todo[0] == 'Park':
                print('Planner - receiving directive from Supervisor to park {0} in Lot {1}'.format(car.name,todo[1]))
                self.cars.append(car)
                create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                self.nursery.start_soon(car.run,send_response_channel,Game)
                end = await self.find_spot_coordinates(todo[1])
                await self.send_directive_to_car(car, end)
                self.assigned.update({car.name: (todo[1])})
                #print(self.parked)
            elif todo == 'Pickup':
                print('Planner -  receiving directive from Supervisor to retrieve {0}'.format(car.name))
                await self.send_directive_to_car(car, PICK_UP)
                self.parked.pop(car.name)

    async def run(self,Game):
        send_response_channel, receive_response_channel = trio.open_memory_channel(25)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.check_supervisor,send_response_channel,Game)
            await trio.sleep(1)
            nursery.start_soon(self.update_car_response,receive_response_channel)
