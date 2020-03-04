import json
from json import JSONEncoder
from jsonschema import validate
import _pickle as pickle
from collections import OrderedDict
from collections import OrderedDict as od
from ipdb import set_trace as st

schema = {
        "type" : "object",
        "properties" : {
            "node_sequence": {"type": "array"},
            "prim_type": {"type": "string"},
            "grid_size": {"type": "number"},
            "start_v": {"type": "number"},
            "end_v": {"type": "number"},
            "test_info": {"type": "array"}
            },
        "required": [
            "node_sequence",
            "prim_type",
            "grid_size",
            "start_v",
            "end_v"
            ]
        }

class WaypointList:
    def __init__(self, l):
        self._list = l

class CustomJSONEncoder(JSONEncoder):
    def default(self, o):
        if isinstance(o, WaypointList):
            return "##<{}>##".format(o._list)


def recompute():
    data = {}
    norm_grid_trajectory_set = [[[0, 0], [1, 0], [2, 0], [3, 0]],
                                [[0, 0], [1, 0], [2, -1], [3, -1]],
                                [[0, 0], [1, 0], [2, 1], [3, 1]],
                                [[0, 0], [1, 0], [2, -1], [2, -2]],
                                [[0, 0], [1, 0], [2, 1], [2, 2]]]

    sim_set = dict()
    sim_set[(0,0)] = [(2.730174518127961, 999.0, 0.5296237328741152), (3.1947179750584196, 999.0, 0.6039733519585504), (2.9600269611982193, 999.0, 0.6854793141852606), (2.946945375575089, 5.600000000000002, 0.5071482908799032), (2.9529104574295957, 999.0, 0.7179876356918411)]
    sim_set[(0,10)] = [(2.771060298574731, 4.800000000000002, 1.0150240121793508), (2.9958146609905723, 5.200000000000002, 1.187321100425719), (2.654892321503462, 5.000000000000002, 1.0783286235294025), (2.468691934613473, 5.000000000000002, 1.1684365223045305), (3.0712091487423554, 4.800000000000002, 1.1053176361027377)]
    sim_set[(10,0)] = [(2.770525158412713, 4.000000000000001, 0.43332719330960906), (3.397514451756727, 999.0, 0.5812877012246122), (3.275513793339767, 4.000000000000001, 0.5135220268209858), (3.1233673682088643, 3.800000000000001, 0.39062929765438076), (3.7439592048396695, 7.200000000000004, 0.44523762219328233)]
    sim_set[(10, 10)] = [(2.6356803875864845, 3.0000000000000004, 1.1170380912780782), (5.104216914134341, 3.2000000000000006, 2.133142862565424), (2.7332871495782167, 3.0000000000000004, 1.0311214654411611), (3.1020801322269205, 3.0000000000000004, 1.1900683266439613), (3.599854643056156, 2.8000000000000003, 1.1365753996863543)]

    all_prims = od()
    grid_size = 10
    prim_id = 0
    for vi in [0, 10]:
        for vf in [0, 10]:
            for sequence_idx, node_sequence in enumerate(norm_grid_trajectory_set):
                prim = od()
                prim['node_sequence'] = WaypointList(node_sequence)
                prim['prim_type'] = 'forward'
                prim['grid_size'] = grid_size
                prim['start_v'] = vi
                prim['end_v'] = vf
                prim['assm'] = sim_set[(vi, vf)][sequence_idx][0]
                prim['guart'] = sim_set[(vi, vf)][sequence_idx][2]
                prim['all_test_info'] = list(sim_set[(vi, vf)][sequence_idx])
                all_prims[prim_id] = prim
                prim_id += 1
    return all_prims



def export_json(data, outfile):
    b = json.dumps(all_prims, indent=2, cls=CustomJSONEncoder)
    b = b.replace('"##<', "").replace('>##"', "")
    with open(outfile, 'w') as f:
        f.write(b)

def import_json(infile):
    with open(infile) as f:
        data = json.load(f, object_pairs_hook=OrderedDict)
    return data


all_prims = recompute()
export_json(all_prims, '10px_prims.json')
my_prims = import_json('10px_prims.json')
for instance in my_prims:
    validate(instance=my_prims[instance], schema=schema)
