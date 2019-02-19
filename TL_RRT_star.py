from Constrees import tree
import numpy as np
from networkx.algorithms import dfs_labeled_edges
import random
import sys
from collections import OrderedDict


def check_subtask(transfer_tree, parent_node, subtask2path, starting2waypoint, sample_list):
    """
    check whether the parent node can be connected to the existing subpath
    :param transfer_tree: 
    :param parent_node: 
    :param subtask2path: 
    :param starting2waypoint:
    :param sample_list:
    :return: 
    """

    # check whether parent node can be connected to existing subpath
    for starting in starting2waypoint.keys():
        if np.linalg.norm(np.subtract(starting[0], parent_node[0])) < 10 and \
                list(transfer_tree.obs_check([parent_node], starting[0], transfer_tree.label(starting[0])).values())[0] \
                and transfer_tree.checkTranB(parent_node[1], transfer_tree.tree.nodes[parent_node]['label'], starting[1]):
            # keep track of the subpath that has been added to the tree, avoid repeating
            # future work, connect added node to the first state of a subpath
            if starting in transfer_tree.used:
                a = (parent_node, starting)
                sample_list.append(a)
                continue
            else:
                transfer_tree.used.add(starting)
                # print(len(trans er_tree.used))
                for subtask in starting2waypoint[starting]:
                    a = subtask2path[subtask].copy()
                    a.insert(0, parent_node)
                    sample_list.append(a)
                    # combine elemental subtask to larger one
                    # sweep_subtask(sample_list, starting2waypoint, subtask2path, transfer_tree)


def sweep_subtask(sample_list, starting2waypoint, subtask2path, transfer_tree):
    """
    comnine subtask to construct complex subtask
    :param sample_list:
    :param starting2waypoint:
    :param subtask2path:
    :param transfer_tree:
    :return:
    """
    for sample in sample_list:
        ending = sample[-1]
        if ending in starting2waypoint.keys() and ending not in transfer_tree.used:
            for subtask in starting2waypoint[ending]:
                a = subtask2path[subtask].copy()
                a.insert(0, sample[-2])
                sample_list.append(a)


def findpath(transfer_tree, goals):
    """
    find the path backwards
    :param transfer_tree
    :param goals: goal state
    :return: dict path : cost
    """
    paths = OrderedDict()
    for i in range(len(goals)):
        true_goal = goals[i][0]
        # true_goal = list(transfer_tree.tree.pred[goal].keys())[0]
        path = [true_goal]
        s = true_goal
        while s != transfer_tree.init:
            s = list(transfer_tree.tree.pred[s].keys())[0]
            if s == path[0]:
                print("loop")
            path.insert(0, s)

        s = goals[i][2]  # accepting point
        suf_path = []  # joint point
        while s != goals[i][1]:
            try:
                s = list(transfer_tree.tree.pred[s].keys())[0]
                suf_path.insert(0, s)
            except IndexError:
                break

        path = path + suf_path
        paths[i] = [transfer_tree.tree.nodes[true_goal]['cost'] + np.linalg.norm(np.subtract(goals[i][0][0], goals[i][1][0])) + \
                    transfer_tree.tree.nodes[goals[i][2]]['cost'] - transfer_tree.tree.nodes[goals[i][1]]['cost'], path]
        # elif transfer_tree.seg == 'suf':
        #     path.append(self.init)
            # paths[i] = [
            #     transfer_tree.tree.nodes[true_goal]['cost'] + np.linalg.norm(np.subtract(true_goal[0], transfer_tree.init[0])),
            #     path]
    return paths


def construction_tree(transfer_tree, buchi_graph, subtask2path, starting2waypoint, uniform):
    """
    constructe the tree in the normal way
    :param transfer_tree: 
    :param buchi_graph: 
    :param subtask2path: 
    :param starting2waypoint:
    :param uniform
    :return: 
    """
    # sample
    x_rand = transfer_tree.sample()
    # nearest
    q_nearest = transfer_tree.nearest(x_rand)
    # steer
    x_new = transfer_tree.steer(x_rand, q_nearest[0][0])
    # label
    label = transfer_tree.label(x_new)
    if 'o' in label:
        return
    if label != '':
        label = label + '_' + str(1)

    # near state
    near_v = transfer_tree.near(x_new)
    # add q_rand
    if q_nearest[0] not in near_v:
        near_v = near_v + q_nearest
    # extend and rewire

    # check obstacle free
    obs_check = transfer_tree.obs_check(near_v, x_new, label)

    sample_list = []
    # iterate over each buchi state
    for b_state in buchi_graph.nodes:

        # new product state
        q_new = (x_new, b_state)

        # extend
        added = transfer_tree.extend(q_new, near_v, label, obs_check)
        
        # rewire
        if added:
            # if 'accept' in q_new[1]:
            #     transfer_tree.goals.append(q_new)
            transfer_tree.rewire(q_new, near_v, obs_check)
            # subpath that can be connected via q_new
            if uniform and random.uniform(0, 1) <= 1:  # float(sys.argv[1]):
                check_subtask(transfer_tree, q_new, subtask2path, starting2waypoint, sample_list)

    return sample_list


def construction_tree_connect_root(transfer_tree, sample_list):
    """
    construct the tree following the existing subpath
    :param transfer_tree: 
    :param sample_list: 
    :return: 
    """
    # extend towards to other sample points
    for k in range(1, len(sample_list)):
        cand = sample_list[k]
        # not in the set of nodes of the tree
        cost = transfer_tree.tree.nodes[sample_list[k - 1]]['cost'] + \
            np.linalg.norm(np.subtract(sample_list[k - 1][0], cand[0]))
        if cand not in transfer_tree.tree.nodes:
            # if 'accept' in cand[1]:
            #     transfer_tree.goals.append(cand)
            label_cand = transfer_tree.label(cand[0])
            transfer_tree.tree.add_node(cand, cost=cost, label=label_cand)
            transfer_tree.tree.nodes[cand]['acc'] = transfer_tree.acpt_check(sample_list[k-1], cand)
            transfer_tree.tree.add_edge(sample_list[k-1], cand)
            transfer_tree.search_goal(cand, label_cand, transfer_tree.tree.nodes[cand]['acc'])
        else:
                delta_c = transfer_tree.tree.nodes[cand]['cost'] - cost
                # update the cost of node in the transfer_tree rooted at sample_list[k]
                if delta_c > 0:
                    if not list(transfer_tree.tree.pred[cand].keys()):
                        print('empty')
                    transfer_tree.tree.remove_edge(list(transfer_tree.tree.pred[cand].keys())[0], cand)
                    transfer_tree.tree.add_edge(sample_list[k-1], cand)
                    transfer_tree.tree.nodes[cand]['acc'] = transfer_tree.acpt_check(sample_list[k-1], cand)
                    edges = dfs_labeled_edges(transfer_tree.tree, source=cand)
                    for u, v, d in edges:
                        if d == 'forward':
                            # update cost
                            transfer_tree.tree.nodes[v]['cost'] = transfer_tree.tree.nodes[v]['cost'] - delta_c
                            # update accept state
                            transfer_tree.tree.nodes[v]['acc'] = transfer_tree.acpt_check(u, v)


def transfer(buchi_graph, ts, no, init, subtask2path, starting2waypoint, n_max, uniform):
    """
    build the tree for the new formula
    :param buchi_graph: 
    :param ts: 
    :param no: 
    :param init: 
    :param subtask2path: 
    :param starting2waypoint:
    :param n_max
    :param uniform
    :return: 
    """
    transfer_tree = tree(ts, buchi_graph, init, 0.25, no)
    # print('--------------prefix path---------------------')
    n_max = n_max
    # for n in range(n_max):
    while 1:
            sample_list = construction_tree(transfer_tree, buchi_graph, subtask2path, starting2waypoint, uniform)
            if sample_list and uniform:
                for i in range(len(sample_list)):
                    # print(sample_list[i])
                    construction_tree_connect_root(transfer_tree, sample_list[i])
            # print(transfer_tree.tree.number_of_nodes())
            if transfer_tree.tree.number_of_nodes() > 5000:
                break
            # if len(transfer_tree.goals) > 0:
            #     print(len(transfer_tree.goals))
            #     break
    path = findpath(transfer_tree, transfer_tree.goals)
    return path, transfer_tree.tree.number_of_nodes()