class BoxComponent:
    def __init__(self):
        self.in_channels = dict()
        self.out_channels = dict()

class CarInfo:
    def __init__(self, arrive_time, depart_time):
        self.arrive_time = arrive_time
        self.depart_time = depart_time

class Time:
    def __init__(self, START_TIME,END_TIME):
        self.START_TIME = START_TIME
        self.END_TIME = END_TIME