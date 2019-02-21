import Buchi
from Problem import problemFormulation
import datetime

# from Biastree4MulR import tree, construction_tree
# from collections import OrderedDict
import numpy as np
# from WorkspacePlot import region_plot, path_plot, layer_plot
# import matplotlib.pyplot as plt
# import networkx as nx
import pickle
# import sys

# +------------------------------------------+
# |     construct transition system graph    |
# +------------------------------------------+

from DetermineRoots import hoftask
from DetectReuse import hoftask_no_simplified, detect_reuse
from Constrees import multi_trees
from TL_RRT_star import transfer
from TransferPlanning import transfer_multi_trees
from Visualization import path_plot

# =========================================================================
# start1 = datetime.datetime.now()
#
# workspace, regions, centers, obs, init_state, uni_cost, formula,\
#             formula_comp, exclusion = problemFormulation(0).Formulation()
# ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}
#
# # # +------------------------------------------+
# # # |            construct buchi graph         |
# # # +------------------------------------------+
#
# buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
# buchi.formulaParser()
# buchi.execLtl2ba()
# _ = buchi.buchiGraph()
# buchi.DelInfesEdge(len(init_state))
# buchi_graph = buchi.buchi_graph
# # roots for reusable skills
# h_task_lib = hoftask((0, 0), buchi_graph, centers)
# time1 = (datetime.datetime.now() - start1).total_seconds()
#
# start2 = datetime.datetime.now()
# end2path = multi_trees(h_task_lib, buchi_graph, ts, centers, 7000)
# time2 = (datetime.datetime.now() - start2).total_seconds()
#
#
# with open('data/lib_subtask', 'wb') as filehandle:
#         # store the data as binary data stream
#         pickle.dump(end2path, filehandle)
#         pickle.dump(h_task_lib, filehandle)

# =====================================================================
# for key, value in end2path.items():
#     path_plot(value, regions, obs)

# # for x in h_task_lib.nodes:
# #     print(x)
# #
# # for e in h_task_lib.edges:
# #     print(e[0], e[1])
# =====================================================================
# new formula
start3 = datetime.datetime.now()
workspace, regions, centers, obs, init_state, uni_cost, formula, \
formula_comp, exclusion = problemFormulation(1).Formulation()
ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}

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
init_pos = (0, 0)
h_task_new = hoftask_no_simplified(init_pos, buchi_graph, centers)
time3 = (datetime.datetime.now() - start3).total_seconds()

# for x in h_task_new.nodes:
#     print(x, x.label)
# for e in h_task_new.edges:
#     print(e[0], e[1])

with open('data/lib_subtask', 'rb') as filehandle:
    # store the data as binary data stream
    end2path = pickle.load(filehandle)
    h_task_lib = pickle.load(filehandle)

subtask2path, starting2waypoint, todo, todo_succ, newsubtask2subtask_p, acpt = detect_reuse(h_task_lib, h_task_new, end2path)
start4 = datetime.datetime.now()

print('time for reusable skills  : %8.2f' % time3)
print('number of reusable skills : %6d' % (len(subtask2path)))
print('number of total subtasks  : %6d' % (h_task_new.number_of_edges()))
print('ratio                     : %8.2f' % (len(subtask2path) / h_task_new.number_of_edges()))

optpath = transfer_multi_trees(buchi_graph, (init_pos, buchi_graph.graph['init'][0]), todo_succ, ts, centers,
                                2000, subtask2path, starting2waypoint, newsubtask2subtask_p, acpt)
time4_transfer = (datetime.datetime.now() - start4).total_seconds()


if optpath:
    path_plot(optpath, regions, obs)
# The ratio varies as different number of iteration to build multitree are used
# for key, value in subtask2path.items():
#     print(key, value)
#
# for key, value in starting2waypoint.items():
#     print(key, ': ', value)

# stats
# n_max = 200
# time_for_new_formula = []
# number_of_paths = []
# optimal_cost = []
# number_of_nodes = []
#
# for k in range(10):
#     # transfer planning ============================================================
#     start4 = datetime.datetime.now()
#     paths_transfer, nn_tranfer = transfer(buchi_graph, ts, no, (init_pos, buchi_graph.graph['init'][0]), subtask2path,
#                                           starting2waypoint, n_max, True)
#     time4_transfer = (datetime.datetime.now() - start4).total_seconds()
#
#     # print('time for reusable skills check:', time3, '\n',
#     print('time for new formula         :', time4_transfer)
#
#     optpath_transfer = []
#     optcost_transfer = np.inf
#     if len(paths_transfer) > 0:
#         for index in paths_transfer.keys():
#             if optcost_transfer > paths_transfer[index][0]:
#                 optcost_transfer = paths_transfer[index][0]
#                 optpath_transfer = paths_transfer[index][1]
#                 # print('number of paths found:', len(paths_transfer), '\n',
#                 #       'optimal cost        :', optcost_transfer, '\n',
#                 #       'number of nodes     :', nn_tranfer)
#                 # path_plot(optpath, regions, obs)
#
#     # if len(optpath_transfer) > 0:
#     #     path_plot(optpath_transfer, regions, obs)
#
#     start5 = datetime.datetime.now()
#     paths, nn = transfer(buchi_graph, ts, no, (init_pos, buchi_graph.graph['init'][0]), dict(), dict(),
#                          n_max, False)
#     time5 = (datetime.datetime.now() - start5).total_seconds()
#
#     # uniform based planning ====================================================
#     optpath = []
#     optcost = np.inf
#     if len(paths) > 0:
#         for index in paths.keys():
#             if optcost > paths[index][0]:
#                 optcost = paths[index][0]
#                 optpath = paths[index][1]
#                 # print('time for uniform sampling:', time5, '\n',
#                 #       'number of paths found:', len(paths), '\n',
#                 #       'optimal cost        :', optcost, '\n',
#                 #       'number of nodes     :', nn)
#                 # print('-----------------------------')
#
#     print('==============================================')
#     print('time for new formula   :  %8.2f | %8.2f' % (time4_transfer, time5))
#     print('number of paths found  :  %6d   | %6d  ' % (len(paths_transfer), len(paths)))
#     print('optimal cost           :  %8.2f | %8.2f' % (optcost_transfer, optcost))
#     print('number of nodes        :  %6d   | %6d   ' % (nn_tranfer, nn))
#
#     # stats
#     time_for_new_formula.insert(k, time4_transfer)
#     time_for_new_formula.append(time5)
#     number_of_paths.insert(k, len(paths_transfer))
#     number_of_paths.append(len(paths))
#     optimal_cost.insert(k, optcost_transfer)
#     optimal_cost.append(optcost)
#     number_of_nodes.insert(k, nn_tranfer)
#     number_of_nodes.append(nn)
#
#     # if len(optpath) > 0:
#     #     path_plot(optpath, regions, obs)
#
# # check wheter a subpath is added to the tree more than once
# print(time_for_new_formula)
# print(number_of_paths)
# print(number_of_nodes)
# print(optimal_cost)
