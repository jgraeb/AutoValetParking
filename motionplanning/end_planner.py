# End planner module
# Tung M. Phan
# California Institute of Technology
# March 5th, 2020
import matplotlib.pyplot as plt
import numpy as np
from ipdb import set_trace as st
from tools import get_rotation_matrix, reflect_over_x_axis, constrain_heading_to_pm_180, img_to_csv_bitmap, get_tube_for_lines, point_set_is_safe

class EndPlanner:
    def __init__(self, planning_graph, end_states, contract):
        self.planning_graph = planning_graph
        self.end_states = end_states
        self.contract = contract
    def get_refined_planning_graph(self):
        for end_state in self.end_states:
            for node in planning_graph['planning_graph']:
                if check_assumption(assume_state=node, end_state=end_state):
                    pass
        return planning_graph

class EndStateContract:
    def __init__(self, assm, guart, bitmap):
        self.assm = assm
        self.guart = guart
        self.bitmap = bitmap.transpose()
    def check_assumption(self, assume_state, end_state):
        if self.assm.check_assumption(assume_state, end_state):
            node_sequence = self.guart.generate_guarantee(assume_state, end_state)
            tube = get_tube_for_lines(node_sequence, r=self.guart.uncertainty)
            return point_set_is_safe(tube, self.bitmap)
        else:
            return False
    def generate_guarantee(self, assume_state, end_state):
        return self.guart.generate_guarantee(assume_state, end_state)

class PrimitiveAssumption:
    def __init__(self, scan_radius):
        self.scan_radius = scan_radius
    def check_scan_radius_constraint(self, assume_state, end_state):
        assume_loc = np.array([[assume_state[0], assume_state[1]]]).transpose()
        end_loc = np.array([[end_state[0], end_state[1]]]).transpose()
        dist = np.linalg.norm(assume_loc - end_loc)
        return dist <= self.scan_radius
    def check_assumption(self, assume_state, end_state):
        raise NotImplementedError

class PrimitiveGuarantee:
    def __init__(self):
        pass
    def generate_guarantee(self, assume_state, end_state):
        raise NotImplementedError

class TwoPointTurnAssumption(PrimitiveAssumption):
    def __init__(self, scan_radius, max_angle_diff, min_dist):
        super(TwoPointTurnAssumption, self).__init__(scan_radius)
        self.max_angle_diff = max_angle_diff
        self.min_dist = min_dist
    def check_assumption(self, assume_state, end_state):
        if self.check_scan_radius_constraint(assume_state, end_state):
            assume_angle = assume_state[2]
            end_angle = end_state[2]
            angle_diff = abs(constrain_heading_to_pm_180(assume_angle-end_angle))
            assume_angle = assume_angle / 180 * np.pi
            end_angle = end_angle / 180 * np.pi
            assume_dir = np.array([[np.cos(assume_angle),-np.sin(assume_angle)]]).transpose()
            end_dir = np.array([[np.cos(end_angle),-np.sin(end_angle)]]).transpose()
            dir_mat = np.hstack((assume_dir, -end_dir))
            assume_loc = np.array([[assume_state[0], assume_state[1]]]).transpose()
            end_loc = np.array([[end_state[0], end_state[1]]]).transpose()
            intersect_params = np.matmul(np.linalg.inv(dir_mat), (end_loc - assume_loc))
            intersect = assume_loc + intersect_params[0] * assume_dir
            dist = np.linalg.norm(end_loc - intersect)
            return angle_diff <= self.max_angle_diff and dist >= self.min_dist
        else:
            return False

class TwoPointTurnGuarantee(PrimitiveGuarantee):
    def __init__(self, uncertainty):
        self.uncertainty = uncertainty
    def generate_guarantee(self, assume_state, end_state):
        assume_angle = assume_state[2]
        end_angle = end_state[2]
        angle_diff = abs(constrain_heading_to_pm_180(assume_angle-end_angle))
        assume_angle = assume_angle / 180 * np.pi
        end_angle = end_angle / 180 * np.pi
        assume_dir = np.array([[np.cos(assume_angle),-np.sin(assume_angle)]]).transpose()
        end_dir = np.array([[np.cos(end_angle),-np.sin(end_angle)]]).transpose()
        dir_mat = np.hstack((assume_dir, -end_dir))
        assume_loc = np.array([[assume_state[0], assume_state[1]]]).transpose()
        end_loc = np.array([[end_state[0], end_state[1]]]).transpose()
        intersect_params = np.matmul(np.linalg.inv(dir_mat), (end_loc - assume_loc))
        intersect = assume_loc + intersect_params[0] * assume_dir
        node_sequence = [[int(assume_loc[0]), int(assume_loc[1])],
                         [int(intersect[0]), int(intersect[1])],
                         [int(end_loc[0]), int(end_loc[1])]]
        return node_sequence

end_states = [(35, 122, 120, 0)]
a = TwoPointTurnAssumption(scan_radius = 200, max_angle_diff=45, min_dist=20)
g = TwoPointTurnGuarantee(uncertainty=3)
bitmap = img_to_csv_bitmap('AVP_planning_300p') # compute bitmap
c = EndStateContract(a, g, bitmap)
c.check_assumption(end_state = end_states[0], assume_state=(20, 10, 90, 0))
#print(a.check_assumption(assume_state=(20, 10, 90, 0), end_state=end_states[0]))
#print(g.generate_guarantee(assume_state=(4, 40, 90, 0), end_state=end_states[0]))
#st()
#            plt.plot(tube[:,0], tube[:,1], '.')
#            plt.axis('equal')
#            plt.gca().invert_xaxis()
#            plt.show()
#            st()
