from components.boxcomponent import BoxComponent
import trio

class Map(BoxComponent): 
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.car_positions = dict()

    async def send_position(self,car):
        print('Map System - sending car position to Planner')
        for key, value in self.car_positions.items():
            if (key==car.name):
                pos = (value.x , value.y, value.yaw, value.v)
        await self.out_channels['Planner'].send((car,pos))

    async def add_car_to_map(self):
        async with self.in_channels['Enter']:
            async for car in self.in_channels['Enter']:
                print('Map System - Adding new car to Map')
                self.car_positions.update({car.name: (car)})
                #print(self.car_positions)

    async def send_camera_data_to_planner(self):
        async with self.in_channels['Planner']:
            async for car in self.in_channels['Planner']:
                await self.send_position(car)
                print(self.car_positions)
            
    async def rmv_car_from_map(self):
        async with self.in_channels['Exit']:
            async for car in self.in_channels['Exit']:
                print('Map System - Removing {} from Map'.format(car.name))
                self.car_positions.pop(car.name)
                #print(self.car_positions)

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.add_car_to_map)
            nursery.start_soon(self.rmv_car_from_map)
            nursery.start_soon(self.send_camera_data_to_planner)
