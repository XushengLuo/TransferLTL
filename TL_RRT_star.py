from Constrees import tree
import numpy as np
from networkx.algorithms import dfs_labeled_edges


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
        if transfer_tree.obs_check([parent_node], starting[0], transfer_tree.label(starting[0]), 'reg')\
                and transfer_tree.checkTranB(parent_node[1], transfer_tree.tree.nodes[parent_node]['label'], starting[1]):
            for subtask in starting2waypoint[starting]:
                a = subtask2path[subtask].copy()
                a.insert(0, parent_node)
                sample_list.append(a)


def construction_tree(transfer_tree, buchi_graph, subtask2path, starting2waypoint):
    """
    constructe the tree in the normal way
    :param transfer_tree: 
    :param buchi_graph: 
    :param subtask2path: 
    :param starting2waypoint: 
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
    obs_check = transfer_tree.obs_check(near_v, x_new, label, 'reg')

    sample_list = []
    # iterate over each buchi state
    for b_state in buchi_graph.nodes:

        # new product state
        q_new = (x_new, b_state)

        # extend
        added = transfer_tree.extend(q_new, near_v, label, obs_check)
        
        # rewire
        if added:
            if 'accept' in q_new[1]:
                transfer_tree.goals.append(q_new)
            transfer_tree.rewire(q_new, near_v, obs_check)
            # subpath that can be connected via q_new
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
            if 'accept' in cand[1]:
                transfer_tree.goals.append(cand)
            transfer_tree.tree.add_node(cand, cost=cost, label=transfer_tree.label(cand[0]))
            transfer_tree.tree.add_edge(sample_list[k-1], cand)
        else:
                delta_c = transfer_tree.tree.nodes[cand]['cost'] - cost
                # update the cost of node in the transfer_tree rooted at sample_list[k]
                if delta_c > 0:
                    if not list(transfer_tree.tree.pred[cand].keys()):
                        print('empty')
                    transfer_tree.tree.remove_edge(list(transfer_tree.tree.pred[cand].keys())[0], cand)
                    transfer_tree.tree.add_edge(sample_list[k-1], cand)
                    edges = dfs_labeled_edges(transfer_tree.tree, source=cand)
                    for _, v, d in edges:
                        if d == 'forward':
                            transfer_tree.tree.nodes[v]['cost'] = transfer_tree.tree.nodes[v]['cost'] - delta_c


def transfer(buchi_graph, ts, no, init, subtask2path, starting2waypoint, n_max):
    """
    build the tree for the new formula
    :param buchi_graph: 
    :param ts: 
    :param no: 
    :param init: 
    :param subtask2path: 
    :param starting2waypoint: 
    :return: 
    """
    transfer_tree = tree('', ts, buchi_graph, init, 'pre', 0.25, no)
    # print('--------------prefix path---------------------')
    n_max = n_max
    for n in range(n_max):
            sample_list = construction_tree(transfer_tree, buchi_graph, subtask2path, starting2waypoint)
            if sample_list:
                for i in range(len(sample_list)):
                    construction_tree_connect_root(transfer_tree, sample_list[i])

    path = transfer_tree.findpath(transfer_tree.goals)
    return path
