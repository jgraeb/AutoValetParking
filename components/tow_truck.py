# Automated Valet Parking - Garage Tow Truck Component
# Josefine Graebener
# California Institute of Technology
# June 2020

from prepare.boxcomponent import BoxComponent
import trio
from variables.global_vars import TOW_TIME

class TowTruck(BoxComponent): 
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__

    async def update_tow_request(self):
        async with self.in_channels['Supervisor']:
            async for car in self.in_channels['Supervisor']:
                print('Towing request received!')
                await trio.sleep(TOW_TIME)
                await self.out_channels['Supervisor'].send(car)

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_tow_request)
