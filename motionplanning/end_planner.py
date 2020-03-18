# End planner module
# Tung M. Phan
# California Institute of Technology
# March 5th, 2020
import _pickle as pickle
import matplotlib.pyplot as plt
import numpy as np
from ipdb import set_trace as st
from motionplanning.tools import constrain_heading_to_pm_180, img_to_csv_bitmap, get_tube_for_lines, point_set_is_safe, compute_sequence_weight, astar_trajectory
import cv2

def find_end_states_from_image(img_path):
    end_states = []
    img = cv2.imread('imglib/{}.png'.format(img_path))
    m = img.shape[0]
    n = img.shape[1]
    for i in range(m):
        for j in range(n):
            b,g,r = img[i][j] # format is BGR, positive integer types!
            if not (r == 255 and g == 255 and b == 255):
                if r == 0:
                    if g == 0:
                        angle = -int(b) # need to recast b to an integer
                        end_states.append((j, i, angle, 0))
                        # reverse parking
#                        ell = 1
#                        i_r, j_r = j + ell * np.cos(angle / 180 * np.pi), i + ell * -np.sin(angle / 180 * np.pi)
#                        end_states.append((i_r, j_r, constrain_heading_to_pm_180(-angle), 0)) # need to recast b to an integer
                    elif b == 0:
                        angle = int(g)
                        end_states.append((j, i, angle, 0))
                        # reverse parking
#                        ell = 1
#                        i_r, j_r = j + ell * np.cos(angle / 180 * np.pi), i + ell * -np.sin(angle / 180 * np.pi)
#                        end_states.append((i_r, j_r, constrain_heading_to_pm_180(-angle), 0))
    print('found {} end states!'.format(len(end_states)))
    return end_states

class EndPlanner:
    def __init__(self, planning_graph, end_states, contract):
        self.graph = planning_graph['graph']
        self.edge_info = planning_graph['edge_info']
        self.end_states = end_states
        self.contract = contract
    def get_planning_graph(self):
        for idx, end_state in enumerate(self.end_states):
            print('planning graph progress: {0:.1f}%'.format(idx/(len(self.end_states)-1)*100))
            for assume_state in self.graph._nodes:
                if self.contract.check_assumption(assume_state=assume_state, end_state=end_state):
                    node_sequence = self.contract.guart.generate_guarantee(assume_state, end_state)
                    self.graph.add_edges([[assume_state, end_state, compute_sequence_weight(node_sequence)]])
                    self.edge_info[assume_state, end_state] = node_sequence
                    if assume_state[3] == 0: # assume state is stopping
                        # add reverse
                        reversed_node_sequence = node_sequence[::-1]
                        self.graph.add_edges([[end_state, assume_state, compute_sequence_weight(reversed_node_sequence)]])
                        self.edge_info[end_state, assume_state] = reversed_node_sequence
        coverage = sum([int(end_state in self.graph._nodes) for end_state in self.end_states])/len(self.end_states)
        print('end state coverage is {0:.1f}%'.format(coverage*100))
        planning_graph['graph'] = self.graph
        planning_graph['edge_info'] = self.edge_info
        return planning_graph

class EndStateContract:
    def __init__(self, assm, guart, bitmap):
        self.assm = assm
        self.guart = guart
        self.bitmap = bitmap.transpose()
    def check_assumption(self, assume_state, end_state):
        try:
            check = self.assm.check_assumption(assume_state, end_state)
            if check:
                node_sequence = self.guart.generate_guarantee(assume_state, end_state)
                tube = get_tube_for_lines(node_sequence, r=self.guart.uncertainty)
                return point_set_is_safe(tube, self.bitmap)
            else:
                return False
        except:
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
            if not (intersect_params[0] >= 0 and intersect_params[1] <= 0):
                return False
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

if __name__ == '__main__':
    remap = False
    if remap:
        end_states = find_end_states_from_image('AVP_planning_300p_end_states')
        assume = TwoPointTurnAssumption(scan_radius=40,max_angle_diff=60,min_dist=15)
        guarantee = TwoPointTurnGuarantee(uncertainty=2)
        bitmap = img_to_csv_bitmap('AVP_planning_300p') # compute bitmap
        contract = EndStateContract(assume, guarantee, bitmap)
        with open('planning_graph.pkl', 'rb') as f:
            planning_graph = pickle.load(f)
        end_planner = EndPlanner(planning_graph=planning_graph,end_states=end_states,contract=contract)
        planning_graph = end_planner.get_planning_graph()
        with open('planning_graph_refined.pkl', 'wb') as f:
            pickle.dump(planning_graph, f)
    else:
        with open('planning_graph_refined.pkl', 'rb') as f:
            planning_graph = pickle.load(f)
        edge_info = planning_graph['edge_info']
        planning_graph = planning_graph['graph']
        ps = []
        ps.append((120, 60, 0, 0))
        plt.plot(ps[0][0], ps[0][1], 'c.')
        img = plt.imread('imglib/AVP_planning_300p.png')
        fig = plt.figure(1)
        plt.imshow(img)
        plt.axis('equal')
        coords = []
        clicks = 0
        print('click on parking lot to set next desired xy')
        clickok = True
        def onclick(event):
            global ix, iy, clicks, coords, ps, clickok
            if clickok:
                clickok = False
                ix, iy = event.xdata, event.ydata
                clicks += 1
                coords.append((ix, iy))
                if clicks % 2: # if odd
                    print('x = %d, y = %d'%( ix, iy))
                    print('click on another point to set desired heading')
                    clickok = True
                else:
                    try:
                        dys = coords[1][1] - coords[0][1]
                        dxs = coords[1][0] - coords[0][0]
                        theta = np.arctan2(-dys, dxs) / np.pi * 180
                        print('theta = %d'%(theta))
                        ps.append((coords[0][0], coords[0][1], theta, 0))
                        coords = []
                        start = ps[-2]
                        end = ps[-1]
                        traj = astar_trajectory(planning_graph, start, end)
                        for start, end in zip(traj, traj[1:]):
                            segment = np.array(edge_info[(tuple(start), tuple(end))])
                            plt.plot(segment[0,0], segment[0,1], 'b.')
                            plt.plot(segment[-1,0], segment[-1,1], 'rx')
                            plt.plot(segment[:,0], segment[:,1], 'k--')
                            plt.pause(0.1)
                        print('trajectory plotted!')
                        print('click to set desired xy')
                        clickok = True
                        plt.show()
                    except:
                        clickok = True
                        print('CANNOT FIND TRAJECTORY: click again to set xy!')
                        if len(ps) > 1:
                            ps = ps[:-1]
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()
