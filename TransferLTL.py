import Buchi
from Problem import problemFormulation
import datetime

# from Biastree4MulR import tree, construction_tree
# from collections import OrderedDict
import numpy as np
# from WorkspacePlot import region_plot, path_plot, layer_plot
# import matplotlib.pyplot as plt
# import networkx as nx
# import pickle
# import sys

# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

from DetermineRoots import hoftask
from DetectReuse import hoftask_no_simplified, detect_reuse
from Constrees import multi_trees
from TL_RRT_star import transfer
from Visualization import path_plot

start1 = datetime.datetime.now()

workspace, regions, centers, obs, init_state, uni_cost, formula,\
            formula_comp, exclusion, no = problemFormulation(0).Formulation()
ts = {'workspace':workspace, 'region':regions, 'obs':obs, 'uni_cost':uni_cost}

# # +------------------------------------------+
# # |            construct buchi graph         |
# # +------------------------------------------+

buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
buchi.formulaParser()
buchi.execLtl2ba()
_ = buchi.buchiGraph()
buchi.DelInfesEdge(len(init_state))
buchi_graph = buchi.buchi_graph
# roots for reusable skills
h_task_lib = hoftask((0, 0), buchi_graph, centers)
time1 = (datetime.datetime.now() - start1).total_seconds()

start2 = datetime.datetime.now()
end2path = multi_trees(h_task_lib, buchi_graph, ts, no, centers)
time2 = (datetime.datetime.now() - start2).total_seconds()


# for x in h_task_lib.nodes:
#     print(x)
#
# for e in h_task_lib.edges:
#     print(e[0], e[1])

# # new formula
start3 = datetime.datetime.now()
workspace, regions, centers, obs, init_state, uni_cost, formula,\
            formula_comp, exclusion, no = problemFormulation(1).Formulation()
ts = {'workspace':workspace, 'region':regions, 'obs':obs, 'uni_cost':uni_cost}

# +------------------------------------------+
# |            construct buchi graph         |
# +------------------------------------------+

buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
buchi.formulaParser()
buchi.execLtl2ba()
_ = buchi.buchiGraph()
buchi.DelInfesEdge(len(init_state))
buchi_graph = buchi.buchi_graph
# roots for new formula, including all possible states
h_task_new = hoftask_no_simplified((0, 0), buchi_graph, centers)
time3 = (datetime.datetime.now() - start3).total_seconds()

# for x in h_task_new.nodes:
#     print(x, x.label)
# for e in h_task_new.edges:
#     print(e[0], e[1])
# reusable

start4 = datetime.datetime.now()
subtask2path, starting2waypoint = detect_reuse(h_task_lib, h_task_new, end2path)

paths = transfer(buchi_graph, ts, no, ((1.0, 0), buchi_graph.graph['init'][0]), subtask2path, starting2waypoint, 150)
time4 = (datetime.datetime.now() - start4).total_seconds()

print(time1, time2, time3, time4)
print(len(paths))
if len(paths) > 0:
    optpath = []
    optcost = np.inf
    for index in paths.keys():
        if optcost > paths[index][0]:
            optcost = paths[index][0]
            optpath = paths[index][1]

    path_plot(optpath, regions, obs)
else:
    print('couldn\' find a feasible path')
# for key, value in subtask2path.items():
#     print(key, value)
#
# for key, value in starting2waypoint.items():
#     print(key, ': ', value)
