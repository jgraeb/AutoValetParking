# Parking lot artifacts, geometries used in AVP layers
from variables.global_vars import  SCALE_FACTOR_PLAN
from variables.parking_data import parking_spots_gazebo_test as parking_spots
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString

##
# Supervisor Data
## [grid_pix(res: 250*250)*SFP=meters]

UPPER_SPOTS = Polygon([(39*SCALE_FACTOR_PLAN, 120*SCALE_FACTOR_PLAN),
                            (106*SCALE_FACTOR_PLAN, 120*SCALE_FACTOR_PLAN),
                            (106*SCALE_FACTOR_PLAN, 175*SCALE_FACTOR_PLAN),
                            (39*SCALE_FACTOR_PLAN, 175*SCALE_FACTOR_PLAN),
                            (39*SCALE_FACTOR_PLAN, 120*SCALE_FACTOR_PLAN)])

LOWER_SPOTS  = Polygon([(39*SCALE_FACTOR_PLAN, 190*SCALE_FACTOR_PLAN),
                            (39*SCALE_FACTOR_PLAN, 240*SCALE_FACTOR_PLAN),
                            (171*SCALE_FACTOR_PLAN, 240*SCALE_FACTOR_PLAN),
                            (171*SCALE_FACTOR_PLAN, 190*SCALE_FACTOR_PLAN),
                            (39*SCALE_FACTOR_PLAN, 190*SCALE_FACTOR_PLAN)])

MIDDLE_BOX  = Polygon([(14*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN),
                            (14*SCALE_FACTOR_PLAN, 230*SCALE_FACTOR_PLAN),
                            (39*SCALE_FACTOR_PLAN, 230*SCALE_FACTOR_PLAN),
                            (39*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN),
                            (14*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN)])
    
##
# Planner Data
## 
LANES_BOX = Polygon([(101,50),(101,230),(181,230),(181,50),(101,50)]) #[grid_pix(res: 250*250)]
        
##
# Camera Data
##
DROPOFF_BOX = Polygon([(83*SCALE_FACTOR_PLAN, 60*SCALE_FACTOR_PLAN),
                            (83*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN),
                            (106*SCALE_FACTOR_PLAN, 60*SCALE_FACTOR_PLAN),
                            (106*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN),
                            (83*SCALE_FACTOR_PLAN, 60*SCALE_FACTOR_PLAN)])

PICKUP_BOX = Point(194*SCALE_FACTOR_PLAN, 60*SCALE_FACTOR_PLAN).buffer(1.0)

PARK_BOXES = [Point(parking_spots[i][0]*SCALE_FACTOR_PLAN,
                    parking_spots[i][1]*SCALE_FACTOR_PLAN).buffer(1.0) for i in list(parking_spots.keys())]
PARK_BOXES_AREA = [Point(parking_spots[i][0]*SCALE_FACTOR_PLAN,
                        parking_spots[i][1]*SCALE_FACTOR_PLAN).buffer(3.0) for i in list(parking_spots.keys())]

FAILURE_ACCEPT_BOX_1 = Polygon([(106*SCALE_FACTOR_PLAN, 71*SCALE_FACTOR_PLAN),
                                    (107*SCALE_FACTOR_PLAN, 133*SCALE_FACTOR_PLAN),
                                    (125*SCALE_FACTOR_PLAN, 133*SCALE_FACTOR_PLAN),
                                    (125*SCALE_FACTOR_PLAN, 158*SCALE_FACTOR_PLAN),
                                    (107*SCALE_FACTOR_PLAN, 158*SCALE_FACTOR_PLAN),
                                    (107*SCALE_FACTOR_PLAN, 202*SCALE_FACTOR_PLAN),
                                    (136*SCALE_FACTOR_PLAN, 203*SCALE_FACTOR_PLAN),
                                    (136*SCALE_FACTOR_PLAN, 227*SCALE_FACTOR_PLAN),
                                    (174*SCALE_FACTOR_PLAN, 227*SCALE_FACTOR_PLAN),
                                    (174*SCALE_FACTOR_PLAN, 71*SCALE_FACTOR_PLAN),
                                    (106*SCALE_FACTOR_PLAN, 71*SCALE_FACTOR_PLAN)])

LANE_1_BOX = Polygon([(106*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN),
                            (141*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN),
                            (141*SCALE_FACTOR_PLAN, 160*SCALE_FACTOR_PLAN),
                            (106*SCALE_FACTOR_PLAN, 160*SCALE_FACTOR_PLAN),
                            (106*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN)])

LANE_2_BOX = Polygon([(141*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN),
                            (141*SCALE_FACTOR_PLAN, 227*SCALE_FACTOR_PLAN),
                            (174*SCALE_FACTOR_PLAN, 227*SCALE_FACTOR_PLAN),
                            (174*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN),
                            (141*SCALE_FACTOR_PLAN, 50*SCALE_FACTOR_PLAN)])