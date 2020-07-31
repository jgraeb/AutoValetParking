# Parking lot artifacts, geometries used in AVP layers
from variables.global_vars import SCALE_FACTOR_PLAN as SFP, SCALE_FACTOR_SIM as SFS, GROUND_PLANE_MPP_LRES
from variables.parking_data import parking_spots
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString

##
# Supervisor Data
## [grid_pix(res: 300*250)*SFP=meters]
UPPER_SPOTS = Polygon([(30*SFP, 120*SFP), (155*SFP, 120*SFP), (155*SFP, 175*SFP), (30*SFP, 175*SFP), (30*SFP, 120*SFP)])
LOWER_SPOTS = Polygon([(30*SFP, 190*SFP), (30*SFP, 240*SFP), (220*SFP, 240*SFP), (220*SFP, 190*SFP), (30*SFP, 190*SFP)])
MIDDLE_BOX = Polygon([(5*SFP,130*SFP),(5*SFP,230*SFP),(30*SFP,230*SFP),(30*SFP,130*SFP),(5*SFP,130*SFP)])

UPPER_SPOTS_GAZEBO = Polygon([(39*GROUND_PLANE_MPP_LRES, 120*GROUND_PLANE_MPP_LRES),
                            (106*GROUND_PLANE_MPP_LRES, 120*GROUND_PLANE_MPP_LRES),
                            (106*GROUND_PLANE_MPP_LRES, 175*GROUND_PLANE_MPP_LRES),
                            (39*GROUND_PLANE_MPP_LRES, 175*GROUND_PLANE_MPP_LRES),
                            (39*GROUND_PLANE_MPP_LRES, 120*GROUND_PLANE_MPP_LRES)])

LOWER_SPOTS_GAZEBO  = Polygon([(39*GROUND_PLANE_MPP_LRES, 190*GROUND_PLANE_MPP_LRES),
                            (39*GROUND_PLANE_MPP_LRES, 240*GROUND_PLANE_MPP_LRES),
                            (171*GROUND_PLANE_MPP_LRES, 240*GROUND_PLANE_MPP_LRES),
                            (171*GROUND_PLANE_MPP_LRES, 190*GROUND_PLANE_MPP_LRES),
                            (39*GROUND_PLANE_MPP_LRES, 190*GROUND_PLANE_MPP_LRES)])

MIDDLE_BOX_GAZEBO  = Polygon([(14*GROUND_PLANE_MPP_LRES,130*GROUND_PLANE_MPP_LRES),
                            (14*GROUND_PLANE_MPP_LRES,230*GROUND_PLANE_MPP_LRES),
                            (39*GROUND_PLANE_MPP_LRES,230*GROUND_PLANE_MPP_LRES),
                            (39*GROUND_PLANE_MPP_LRES,130*GROUND_PLANE_MPP_LRES),
                            (14*GROUND_PLANE_MPP_LRES,130*GROUND_PLANE_MPP_LRES)])
    
##
# Planner Data
## 
LANES_BOX = Polygon([(150,50),(150,230),(230,230),(230,50),(150,50)]) #[grid_pix(res: 300*250)]
LANES_BOX_GAZEBO = Polygon([(101,50),(101,230),(181,230),(181,50),(101,50)])
        
##
# Camera Data
##
DROPOFF_BOX = Polygon([(40, 18), (40, 15), (47, 18), (47, 15), (40, 18)])
PICKUP_BOX = Point(260*SFP, 60*SFP).buffer(1.0)
PARK_BOXES = [Point(parking_spots[i][0]*SFP,parking_spots[i][1]*SFP).buffer(1.0) for i in list(parking_spots.keys())]
PARK_BOXES_AREA = [Point(parking_spots[i][0]*SFP,parking_spots[i][1]*SFP).buffer(3.0) for i in list(parking_spots.keys())]
FAILURE_ACCEPT_BOX_1 = Polygon([(155*SFP, 71*SFP), (156*SFP, 133*SFP), (174*SFP, 133*SFP), (174*SFP, 158*SFP), (156*SFP, 158*SFP),
                                (156*SFP, 202*SFP), (185*SFP, 203*SFP), (185*SFP, 227*SFP), (223*SFP, 227*SFP), (223*SFP, 71*SFP),
                                (155*SFP, 71*SFP)])
LANE_1_BOX = Polygon([(155*SFP, 50*SFP), (190*SFP, 50*SFP), (190*SFP, 160*SFP), (155*SFP, 160*SFP), (155*SFP, 50*SFP)])
LANE_2_BOX = Polygon([(190*SFP, 50*SFP), (190*SFP, 227*SFP), (223*SFP, 227*SFP), (223*SFP, 50*SFP), (190*SFP, 50*SFP)])


DROPOFF_BOX_GAZEBO = Polygon([(83*GROUND_PLANE_MPP_LRES, 60*GROUND_PLANE_MPP_LRES),
                            (83*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES),
                            (106*GROUND_PLANE_MPP_LRES, 60*GROUND_PLANE_MPP_LRES),
                            (106*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES),
                            (83*GROUND_PLANE_MPP_LRES, 60*GROUND_PLANE_MPP_LRES)])

PICKUP_BOX_GAZEBO = Point(194*GROUND_PLANE_MPP_LRES, 60*GROUND_PLANE_MPP_LRES).buffer(1.0)

PARK_BOXES_GAZEBO = [Point(parking_spots[i][0]*GROUND_PLANE_MPP_LRES,
                    parking_spots[i][1]*GROUND_PLANE_MPP_LRES).buffer(1.0) for i in list(parking_spots.keys())]
PARK_BOXES_AREA_GAZEBO = [Point(parking_spots[i][0]*GROUND_PLANE_MPP_LRES,
                        parking_spots[i][1]*GROUND_PLANE_MPP_LRES).buffer(3.0) for i in list(parking_spots.keys())]

FAILURE_ACCEPT_BOX_1_GAZEBO = Polygon([(106*SFP, 71*SFP),
                                    (107*GROUND_PLANE_MPP_LRES, 133*GROUND_PLANE_MPP_LRES),
                                    (125*GROUND_PLANE_MPP_LRES, 133*GROUND_PLANE_MPP_LRES),
                                    (125*GROUND_PLANE_MPP_LRES, 158*GROUND_PLANE_MPP_LRES),
                                    (107*GROUND_PLANE_MPP_LRES, 158*GROUND_PLANE_MPP_LRES),
                                    (107*GROUND_PLANE_MPP_LRES, 202*GROUND_PLANE_MPP_LRES),
                                    (136*GROUND_PLANE_MPP_LRES, 203*GROUND_PLANE_MPP_LRES),
                                    (136*GROUND_PLANE_MPP_LRES, 227*GROUND_PLANE_MPP_LRES),
                                    (174*GROUND_PLANE_MPP_LRES, 227*GROUND_PLANE_MPP_LRES),
                                    (174*GROUND_PLANE_MPP_LRES, 71*GROUND_PLANE_MPP_LRES),
                                    (106*GROUND_PLANE_MPP_LRES, 71*GROUND_PLANE_MPP_LRES)])

LANE_1_BOX_GAZEBO = Polygon([(106*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES),
                            (141*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES),
                            (141*GROUND_PLANE_MPP_LRES, 160*GROUND_PLANE_MPP_LRES),
                            (106*GROUND_PLANE_MPP_LRES, 160*GROUND_PLANE_MPP_LRES),
                            (106*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES)])

LANE_2_BOX_GAZEBO = Polygon([(141*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES),
                            (141*GROUND_PLANE_MPP_LRES, 227*GROUND_PLANE_MPP_LRES),
                            (174*GROUND_PLANE_MPP_LRES, 227*GROUND_PLANE_MPP_LRES),
                            (174*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES),
                            (141*GROUND_PLANE_MPP_LRES, 50*GROUND_PLANE_MPP_LRES)])