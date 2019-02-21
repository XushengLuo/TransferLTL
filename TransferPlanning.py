import numpy as np
from datetime import datetime
from Constrees import tree
import random
from networkx.algorithms import dfs_labeled_edges
from DetectReuse import replace
from collections import OrderedDict


def path_cost(path):
    """
    calculate cost
    :param path:
    :return:
    """
    cost = 0
    for k in range(len(path) - 1):
        cost += np.linalg.norm(np.subtract(path[k + 1][0], path[k][0]))
    return cost


def path_root(subtree, curr, succ):
    """
    find the path to successor of current root
    :param subtree:
    :param curr:
    :param succ:
    :return:
    """
    goal = succ
    path = [goal]
    s = goal
    while s != curr:
        s = list(subtree.tree.pred[s].keys())[0]
        if s == path[0]:
            print("loop")
        path.insert(0, s)

    return path


def update(curr, succ, path, starting2waypoint, subtask2path, newsubtask2subtask_p):
    """
    add new found subtask to library
    :param curr:
    :param succ:
    :param path:
    :param starting2waypoint:
    :param subtask2path:
    :param newsubtask2subtask_p:
    :return:
    """
    subtask2path[(curr, succ)] = path
    # curr, succ may be the init root to any other root
    if (curr, succ) not in newsubtask2subtask_p.keys():
        return
    for subtask in newsubtask2subtask_p[(curr, succ)]:
        # replace
        if subtask[2] == 'forward':
            subtask2path[(subtask[0], subtask[1])] \
                = replace(subtask[0][1], subtask[1][1], curr[1], succ[1], path, 'forward')
        elif subtask[2] == 'backward':
            subtask2path[(subtask[0], subtask[1])] \
                = replace(subtask[0][1], subtask[1][1], curr[1], succ[1], path, 'backward')
        # add to starting2waypoint, it's possible that subtask[0] is not in the key list
        if subtask[0] in starting2waypoint.keys():
            starting2waypoint[subtask[0]].add((subtask[0], subtask[1]))
        else:
            starting2waypoint[subtask[0]] = {(subtask[0], subtask[1])}


def check_subtask(subtree, parent_node, subtask2path, starting2waypoint, sample_list):
    """
    check whether the parent node can be connected to the existing subpath
    :param subtree: 
    :param parent_node: 
    :param subtask2path: 
    :param starting2waypoint:
    :param sample_list:
    :return: 
    """

    # check whether parent node can be connected to existing subpath
    # obs_check: return a dictionary
    for starting in starting2waypoint.keys():
        if np.linalg.norm(np.subtract(starting[0], parent_node[0])) < 0.2 and \
                list(subtree.obs_check([parent_node], starting[0], subtree.label(starting[0])+'_'+str(1)).values())[0] \
                and subtree.checkTranB(parent_node[1], subtree.tree.nodes[parent_node]['label'], starting[1]):
            # keep track of the subpath that has been added to the tree, avoid repeating
            # future work, connect added node to the first state of a subpath
            if starting in subtree.used:
                a = (parent_node, starting)
                sample_list.append(a)
                continue
            else:
                subtree.used.add(starting)
                # print(len(trans er_tree.used))
                for subtask in starting2waypoint[starting]:
                    a = subtask2path[subtask].copy()
                    a.insert(0, parent_node)
                    sample_list.append(a)
                    # combine elemental subtask to larger one

                    # sweep_subtask(sample_list, starting2waypoint, subtask2path, subtree)


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

def update_cost_acc(multi_tree, subtree, succ, changed, index2node_root, root2index, root_pred2index_node):
    """
    update cost and acc of all children node in all subtrees
    :param multi_tree:
    :param subtree:
    :param succ:
    :param changed:
    :param index2node_root:
    :param root2index:
    :param root_pred2index_node:
    :return:
    """
    # we only update the cost and acc if subtree connects to the root of the whole formula
    if subtree.tree.nodes[succ]['cost'] < subtree.base:
        # all other roots in the current tree or its subtrees
        to_update = [root2index[succ]]
        while to_update:
            index = to_update.pop(0)
            to_update += [root2index[r[1]] for r in index2node_root[index]]
            root = multi_tree[index].init
            multi_tree[index].tree.nodes[root]['acc'] = set(multi_tree[root_pred2index_node[root][0]].tree.nodes[root][
                                                                'acc'])
            delta_c = multi_tree[index].tree.nodes[root]['cost'] - \
                      multi_tree[root_pred2index_node[root][0]].tree.nodes[root][
                          'cost']
            for nodes in multi_tree[index].tree.nodes():
                multi_tree[index].tree.nodes[nodes]['cost'] -= delta_c

            if changed:
                for u, v, d in dfs_labeled_edges(multi_tree[index].tree, source=root):
                    if d == 'forward':
                        multi_tree[index].tree.nodes[v]['acc'] = set(multi_tree[index].acpt_check(u, v)[0])


def search_goal(multitree, init, subtree, q_new, label_new, acc, root_pred2index_node, root2index):
    """

    :param multitree:
    :param init:
    :param subtree:
    :param q_new:
    :param label_new:
    :param acc:
    :param root_pred2index_node:
    :param root2index:
    :return:
    """
    # print(subtree.tree.nodes[q_new]['cost'])
    if subtree.tree.nodes[q_new]['cost'] < subtree.base:
        currtree = multitree[root2index[subtree.init]]
        path = [root2index[currtree.init]] + currtree.findpath([q_new])[0][1]

        while path[1] != init:
            index, parentnode = root_pred2index_node[currtree.init]
            currtree = multitree[index]
            path = [index] + currtree.findpath([parentnode])[0][1] + path
            # print(path)
        index = 0  # initialize
        currtree = multitree[index]
        for ac in acc:
            # connect to path leading to accepting state, including accepting state
            try:
                ac_index = path.index(ac)
            except ValueError:
                # path has changed leading to current node
                continue
            for point in path[:ac_index + 1]:
                if isinstance(point, int):
                    index = point
                    currtree = multitree[index]
                    continue
                # point in currtree
                if list(currtree.obs_check([point], q_new[0], label_new).values())[0] \
                        and currtree.checkTranB(q_new[1], label_new, point[1]):
                    print('+1', np.sum([t.tree.number_of_nodes() for t in multitree]))
                    subtree.goals.append((q_new, point, ac))  # endpoint, middle point, accepting point


def find_path(multitree, init, subtree, goal, root_pred2index_node, root2index):
    """
    find the path for a goal point
    :param multitree:
    :param init:
    :param subtree:
    :param goal: (endpoint, middle point, accepting point)
    :param goal_new:
    :param goal_acc:
    :param root_pred2index_node:
    :param root2index:
    :return:
    """
    currtree = multitree[root2index[subtree.init]]
    path = currtree.findpath([goal[0]])[0][1]
    while path[0] != init:
        index, parentnode = root_pred2index_node[currtree.init]
        currtree = multitree[index]
        path = currtree.findpath([parentnode])[0][1] + path
    try:
        path = path + path[path.index(goal[1]):path.index(goal[2]) + 1]
    except ValueError:
        print('empty')
        return []
    return path


def construction_tree(subtree, x_rand, buchi_graph, centers, todo_succ, flag, connect, subtask2path, starting2waypoint,
                      newsubtask2subtask_p, root2index, root_pred2index_node, index2node_root, multi_tree, init, pred):
    """
    construct one specific subtree
    :param subtree: 
    :param x_rand: 
    :param buchi_graph: 
    :param centers:
    :param todo_succ: successor of new subtask
    :param flag: connect to other roots or not
    :param connect: connected pairs of roots
    :param subtask2path: 
    :param starting2waypoint: 
    :param newsubtask2subtask_p:
    :param root2index:
    :param root_pred2index_node:
    :param index2node_root:
    :param multi_tree:
    :param init: init root of the whole formula
    :return: 
    """
    # # sample
    # x_rand = subtree.sample()
    # nearest
    q_nearest = subtree.nearest(x_rand)
    # steer
    x_new = subtree.steer(x_rand, q_nearest[0][0])
    # label of q_new
    label = subtree.label(x_new)
    if 'o' in label:
        return
    if label != '':
        label = label + '_' + str(1)

    # near state
    near_v = subtree.near(x_new) + pred
    # add q_rand
    if q_nearest[0] not in near_v:
        near_v = near_v + q_nearest
    # extend and rewire

    # check obstacle free
    obs_check = subtree.obs_check(near_v, x_new, label)

    sample_list = []
    # iterate over each buchi state
    for b_state in buchi_graph.nodes:

        # new product state
        q_new = (x_new, b_state)

        # extend
        # no need to consider update the cost and acc of other subtrees since connect_root handles it
        # succ = set(todo_succ.keys())
        # succ.remove(subtree.init)
        succ = set(todo_succ[subtree.init])
        added = subtree.extend(q_new, near_v, label, obs_check, succ)
        # rewire
        if added:
            search_goal(multi_tree, init, subtree, q_new, label, subtree.tree.nodes[q_new]['acc'], root_pred2index_node,
                        root2index)
            # only rewire within the subtree, it may affect the node which is a root
            subtree.rewire(q_new, near_v, obs_check)
            if random.uniform(0, 1) <= 1:  # float(sys.argv[1]):
                check_subtask(subtree, q_new, subtask2path, starting2waypoint, sample_list)
            # if extend successfullly
            if flag:
                construction_tree_connect_root(subtree, q_new, label, centers, todo_succ, connect, starting2waypoint,
                                               subtask2path, newsubtask2subtask_p, root2index, root_pred2index_node,
                                               index2node_root, multi_tree, init)

    return sample_list


def construction_tree_connect_root(subtree, q_new, label, centers, todo_succ, connect, starting2waypoint, subtask2path,
                                   newsubtask2subtask_p, root2index, root_pred2index_node, index2node_root, multi_tree,
                                   init):
    """
    connect current subtree to other roots
    :param subtree:
    :param q_new:
    :param label:
    :param centers:
    :param todo_succ:
    :param connect:
    :param starting2waypoint:
    :param subtask2path:
    :param newsubtask2subtask_p:
    :param root2index:
    :param root_pred2index_node:
    :param index2node_root:
    :param multi_tree:
    :return:
    """
    # if subtree.tree.graph['init'] == init:
    #     return
    # extend towards to other roots
    curr = subtree.init
    if curr == init:
        succ_list = todo_succ.keys()
    else:
        succ_list = todo_succ[curr]
    for succ in succ_list:
        if succ == curr:
            continue
        # label of succ
        label_succ = ''
        for l, coord in centers.items():
            if coord == succ[0]:
                label_succ = l + '_' + str(1)
                break

        # connect q_new to root succ
        if (list(subtree.obs_check([q_new], succ[0], label_succ).values())[0] and subtree.checkTranB(q_new[1], label,
                                                                                succ[1])):  # or q_new == succ:
            #                                                            equivalent nodes
            c = subtree.tree.nodes[q_new]['cost'] + \
                np.linalg.norm(np.subtract(q_new[0], succ[0]))
            acc, changed = subtree.acpt_check(q_new, succ)
            # in the tree
            if succ in subtree.tree.nodes():
                delta_c = subtree.tree.nodes[succ]['cost'] - c
                # update the cost of node in the subtree rooted at near_vertex
                if delta_c > 0:
                    subtree.tree.remove_edge(list(subtree.tree.pred[succ].keys())[0], succ)
                    subtree.tree.add_edge(q_new, succ)
                    subtree.tree.nodes[succ]['cost'] = c  # two kinds of succ, one in tree, one the root
                    subtree.tree.nodes[succ]['acc'] = set(acc)

                    # update lib
                    path = path_root(subtree, curr, succ)
                    update(curr, succ, path, starting2waypoint, subtask2path, newsubtask2subtask_p)

                    # succ has been added to one certain subtree
                    # succ in the current subtree < succ in the original subtree which connects to succ, the subtree: rewire
                    if c < multi_tree[root_pred2index_node[succ][0]].tree.nodes[succ]['cost']:
                        # remove
                        index, node = root_pred2index_node[succ]
                        index2node_root[index].remove((node, succ))
                        # add
                        index2node_root[root2index[subtree.init]].add((q_new, succ))
                        root_pred2index_node[succ] = [root2index[subtree.init], q_new]
                        # update the cost and acc of curr and all nodes in the subtrees of curr
                        update_cost_acc(multi_tree, subtree, succ, changed, index2node_root, root2index,
                                        root_pred2index_node)
                        # !!!! future work: update subtasks newly added to the library periodically !!!!

            # not in the tree
            else:
                subtree.tree.add_node(succ, cost=c, label=label_succ, acc=set(acc))
                subtree.tree.add_edge(q_new, succ)
                # keep track of connection
                connect.add((curr, succ))
                # update lib of subtask
                path = path_root(subtree, curr, succ)
                # add to starting2waypoint and subtask2path
                update(curr, succ, path, starting2waypoint, subtask2path, newsubtask2subtask_p)

                if succ not in root_pred2index_node.keys():
                    # succ has not been added to existing subtrees
                    # add to the list of roots in the current subtree
                    index2node_root[root2index[subtree.init]].add((q_new, succ))
                    root_pred2index_node[succ] = [root2index[subtree.init], q_new]

                    # update the cost and acc of curr and all nodes in the subtree of curr
                    update_cost_acc(multi_tree, subtree, succ, changed, index2node_root, root2index,
                                    root_pred2index_node)

                else:
                    # this part is identical to the case where curr is in the subtree already
                    # succ has been added to one subtree
                    # succ in the current subtree < succ in the original subtree which connects to succ, the subtree
                    if c < multi_tree[root_pred2index_node[succ][0]].tree.nodes[succ]['cost']:
                        # remove
                        index, node = root_pred2index_node[succ]
                        index2node_root[index].remove((node, succ))
                        # add
                        index2node_root[root2index[subtree.init]].add((q_new, succ))
                        root_pred2index_node[succ] = [root2index[subtree.init], q_new]
                        # update the cost and acc of curr and all nodes in the subtree of curr
                        update_cost_acc(multi_tree, subtree, succ, changed, index2node_root, root2index,
                                        root_pred2index_node)


def construction_tree_connect_sample(subtree, sample_list, multi_tree, init, root_pred2index_node, root2index, centers,
                                     todo_succ, connect, starting2waypoint, subtask2path, newsubtask2subtask_p,
                                     index2node_root):
    """
    construct current subtree following the existing subpath
    :param subtree:
    :param sample_list:
    :param multi_tree:
    :param init:
    :param root_pred2index_node:
    :param root2index:
    :return:
    """

    # extend towards to other sample points
    for k in range(1, len(sample_list)):
        cand = sample_list[k]
        # connect the second to last to other roots
        if k == len(sample_list)-1:
            construction_tree_connect_root(subtree, sample_list[k - 1], subtree.tree.nodes[sample_list[k - 1]]['label'],
                                            centers, todo_succ, connect,
                                            starting2waypoint, subtask2path, newsubtask2subtask_p, root2index,
                                            root_pred2index_node, index2node_root, multi_tree, init)
            break

        # not in the set of nodes of the tree
        cost = subtree.tree.nodes[sample_list[k - 1]]['cost'] + \
               np.linalg.norm(np.subtract(sample_list[k - 1][0], cand[0]))
        if cand not in subtree.tree.nodes():
            # extend-like
            # if 'accept' in cand[1]:
            #     subtree.goals.append(cand)
            label_cand = subtree.label(cand[0]) + '_' + str(1)
            subtree.tree.add_node(cand, cost=cost, label=label_cand)
            subtree.tree.nodes[cand]['acc'] = set(subtree.acpt_check(sample_list[k - 1], cand)[0])
            subtree.tree.add_edge(sample_list[k - 1], cand)
            search_goal(multi_tree, init, subtree, cand, label_cand, subtree.tree.nodes[cand]['acc'],
                        root_pred2index_node, root2index)

        else:
            # rewire-like
            delta_c = subtree.tree.nodes[cand]['cost'] - cost
            # update the cost of node in the subtree rooted at sample_list[k]
            if delta_c > 0:
                subtree.tree.remove_edge(list(subtree.tree.pred[cand].keys())[0], cand)
                subtree.tree.add_edge(sample_list[k - 1], cand)
                acc, changed = subtree.acpt_check(sample_list[k - 1], cand)
                subtree.tree.nodes[cand]['acc'] = set(acc)
                edges = dfs_labeled_edges(subtree.tree, source=cand)
                for u, v, d in edges:
                    if d == 'forward':
                        # update cost
                        subtree.tree.nodes[v]['cost'] = subtree.tree.nodes[v]['cost'] - delta_c
                        # update accept state
                        if changed:
                            subtree.tree.nodes[v]['acc'] = set(subtree.acpt_check(u, v)[0])

                            # those sampled point are more likely to be specific to one task, and the endpoint of the subpath may not
                            # directly connect to other roots since two regions can not be connected via a straight line
                            # connect to other roots


def transfer_multi_trees(buchi_graph, init, todo_succ, ts, centers, max_node, subtask2path, starting2waypoint,
                         newsubtask2subtask_p, acpt):
    """
    build multiple subtree by transferring
    :param todo: new subtask built from the scratch
    :param buchi_graph: 
    :param init: root for the whole formula
    :param todo_succ: the successor of new subtask
    :param ts: 
    :param no: 
    :param centers: 
    :param max_node: maximum number of nodes of all subtrees
    :param subtask2path: (init, end) --> init, p, ..., p, end
    :param starting2waypoint: init --> (init, end)
    :param newsubtask2subtask_p: new subtask need to be planned --> new subtask noneed
    :return: 
    """

    multi_tree = list()
    # a list of subtrees
    # for root in h_task.nodes:
    #     if 'accept' not in root.q:
    #         init = (root.x, root.q)
    init_root = False
    root_index = 0
    root2index = dict()  # root --> index in the list of roots
    root_pred2index_node = dict()  # root --> pred node and index
    base = 1e3
    # todo_succ.values include key waypoint but not new root
    for td in todo_succ.keys():
        multi_tree.append(tree(ts, buchi_graph, td, 0.25, base))
        if init == td:
            # the base for the init root is 0
            init_root = True
        else:
            multi_tree[-1].tree.nodes[td]['cost'] = base
        root2index[td] = root_index
        root_index += 1

    if not init_root:
        multi_tree.append(tree(ts, buchi_graph, init, 0.25, base))
        root2index[init] = root_index
        root_index += 1
    # accept
    # for ac in acpt:
    #     if ac not in root2index.keys():
    #         print(ac)
    #         multi_tree.append(tree(ts, buchi_graph, ac, 0.25, base))
    #         multi_tree[-1].tree.nodes[ac]['cost'] = base
    #         root2index[ac] = root_index
    #         root_index += 1

    index2node_root = {i: set() for i in range(len(multi_tree))}  # index -->  node and successive root
    print('number of subtress        : %8d' % len(multi_tree))

    c = 0
    connect = set()
    # for n in range(n_max):
    now = datetime.now()
    while np.sum([t.tree.number_of_nodes() for t in multi_tree]) < max_node:
        x_rand = multi_tree[0].sample()
        for i in range(len(multi_tree)):
            if np.sum([t.tree.number_of_nodes() for t in multi_tree]) < c * max_node:  # 0 is better
                sample_list = construction_tree(multi_tree[i], x_rand, buchi_graph, centers, todo_succ, 0, connect,
                                                subtask2path, starting2waypoint, newsubtask2subtask_p, root2index,
                                                root_pred2index_node, index2node_root, multi_tree, init, [])
            else:
                sample_list = construction_tree(multi_tree[i], x_rand, buchi_graph, centers, todo_succ, 1, connect,
                                                subtask2path, starting2waypoint, newsubtask2subtask_p, root2index,
                                                root_pred2index_node, index2node_root, multi_tree, init, [])
            if sample_list:
                for sample in sample_list:
                    construction_tree_connect_sample(multi_tree[i], sample, multi_tree, init,
                                                     root_pred2index_node, root2index, centers,
                                                     todo_succ, connect, starting2waypoint, subtask2path,
                                                     newsubtask2subtask_p, index2node_root)
                    # for s in range(len(sample)-1):
                    #     sample_list += construction_tree(multi_tree[i], sample[s+1][0], buchi_graph, centers, todo_succ, 0, connect,
                    #                                 subtask2path, starting2waypoint, newsubtask2subtask_p, root2index,
                    #                                 root_pred2index_node, index2node_root, multi_tree, init, [sample[s]])
        # roots connected to init root
        # for root in root2index.keys():
        #     index = [root2index[nr[1]] for nr in index2node_root[root2index[root]]]
        #     if index:
        #         print(root2index[init])
        #         print(root2index[root], index)
        #         print(root2index[root], [root_pred2index_node[multi_tree[i].init][0] for i in index])
        # print('===================')
    time2 = (datetime.now() - now).total_seconds()
    print(time2, np.sum([len(v) for k, v in todo_succ.items()]), len(connect))
    # for c in connect:
    #     print(c)
    # print('=======================')
    # for k, v in todo_succ.items():
    #     print(k, v)

    print(np.sum([len(t.goals) for t in multi_tree]))

    paths = OrderedDict()
    k = 0
    optcost = np.inf
    optpath = []
    for t in multi_tree:
        for goal in t.goals:
            path = find_path(multi_tree, init, t, goal, root_pred2index_node, root2index)
            # something happens to the original path
            if not path:
                continue
            paths[k] = path
            # update optimal path
            c = path_cost(path)
            if c < optcost:
                optcost = c
                optpath = path
            k += 1
    # print(time1, time2, num_path_seq, num_path_par)
    return optpath
