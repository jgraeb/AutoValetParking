# Parking lot artifacts, geometries used in AVP layers
from variables.global_vars import SCALE_FACTOR_PLAN as SFP, SCALE_FACTOR_SIM as SFS
from variables.parking_data import parking_spots
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString

##
# Supervisor Data
##
UPPER_SPOTS = Polygon([(30*SFP, 120*SFP), (155*SFP, 120*SFP), (155*SFP, 175*SFP), (30*SFP, 175*SFP), (30*SFP, 120*SFP)])
LOWER_SPOTS = Polygon([(30*SFP, 190*SFP), (30*SFP, 240*SFP), (220*SFP, 240*SFP), (220*SFP, 190*SFP), (30*SFP, 190*SFP)])
MIDDLE_BOX = Polygon([(5*SFP,130*SFP),(5*SFP,230*SFP),(30*SFP,230*SFP),(30*SFP,130*SFP),(5*SFP,130*SFP)])
    
##
# Planner Data
##
LANES_BOX = Polygon([(150,50),(150,230),(230,230),(230,50),(150,50)])
        
##
# Camera Data
##
DROPOFF_BOX = Polygon([(40, 18), (40, 15), (47, 18), (47, 15), (40, 18)])
PICKUP_BOX = Point(260*SFP, 60*SFP).buffer(1.0)
PARK_BOXES = [Point(parking_spots[i][0]*SFP,parking_spots[i][1]*SFP).buffer(1.0) for i in list(parking_spots.keys())]
PARK_BOXES_AREA = [Point(parking_spots[i][0]*SFP,parking_spots[i][1]*SFP).buffer(3.0) for i in list(parking_spots.keys())]
FAILURE_ACCEPT_BOX_1 = Polygon([(155*SFP, 71*SFP), (156*SFP, 133*SFP), (174*SFP, 133*SFP), (174*SFP, 158*SFP), (156*SFP, 158*SFP), (156*SFP, 202*SFP), (185*SFP, 203*SFP), (185*SFP, 227*SFP), (223*SFP, 227*SFP), (223*SFP, 71*SFP), (155*SFP, 71*SFP)])
LANE_1_BOX = Polygon([(155*SFP, 50*SFP), (190*SFP, 50*SFP), (190*SFP, 160*SFP), (155*SFP, 160*SFP), (155*SFP, 50*SFP)])
LANE_2_BOX = Polygon([(190*SFP, 50*SFP), (190*SFP, 227*SFP), (223*SFP, 227*SFP), (223*SFP, 50*SFP), (190*SFP, 50*SFP)])
