from components.boxcomponent import BoxComponent
import trio
import sys
sys.path.append('..') # enable importing modules from an upper directory
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from prepare.helper import draw_car, draw_pedestrian
from component import parking_lot
from variables.global_vars import *
from variables.data import parking_spots

show_all_spots = True

class Simulation(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        self.ax = []
        self.fig = []
        self.background = []
        self.start_walk_lane = (2908,665)
        self.end_walk_lane = (3160,665)
    
    async def add_car_to_sim(self):
        while True:
            async with self.in_channels['Game']:
                async for car in self.in_channels['Game']:
                    print('Simulation System - Adding new car to Map')
                    self.cars.append(car)

    async def add_ped_to_sim(self):
        while True:
            async with self.in_channels['PedSimulation']:
                async for ped in self.in_channels['PedSimulation']:
                    print('Simulation System - Adding new pedestrian to Map')
                    self.peds.append(ped)
                    #print(self.peds)

    def animate(self, frame_idx): # update animation by dt
        self.ax.clear()
        # scale to the large topo
        xoffset = 0
        yoffset = 200
        for pedestrian in self.peds:
            draw_pedestrian(pedestrian,self.background)
        for car in self.cars:
            draw_car(self.background, car.x*SCALE_FACTOR_SIM+xoffset,car.y*SCALE_FACTOR_SIM+yoffset,car.yaw)
        # to check parking spot locations
        if show_all_spots:
            for key,value in parking_spots.items():
                draw_car(self.background,  value[0]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM+xoffset, value[1]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM+yoffset,np.deg2rad(value[2]))
        the_parking_lot = [self.ax.imshow(self.background)] # update the stage
        self.background.close()
        self.background = parking_lot.get_background()
        all_artists = the_parking_lot
        return all_artists

    async def update_simulation(self):
        self.fig = plt.figure()
        while True:
            self.ax = self.fig.add_axes([0,0,1,1]) # get rid of white border
            plt.axis('off')
            self.background = parking_lot.get_background()
            ani = animation.FuncAnimation(self.fig, self.animate, frames=1, interval=1**3, blit=True, repeat=False)
            plt.pause(0.001)
            plt.draw()
            await trio.sleep(0)
            print('------------Figure updating-------------')

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.add_car_to_sim)
            await trio.sleep(0)
            nursery.start_soon(self.add_ped_to_sim)
            await trio.sleep(0)
            nursery.start_soon(self.update_simulation)
            await trio.sleep(0)