"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 8/30/18
"""
"""
construct trees for biased sampling optimal task planning for multi-robots
"""

from random import uniform
from networkx.classes.digraph import DiGraph
from networkx.algorithms import dfs_labeled_edges
import math
import numpy as np
from collections import OrderedDict
from shapely.geometry import Point, Polygon, LineString
from datetime import datetime
import random


class tree(object):
    """ construction of prefix and suffix tree
    """
    def __init__(self, ts, buchi_graph, init, step_size, base=1e3):
        """
        :param ts: transition system
        :param buchi_graph:  Buchi graph
        :param init: product initial state
        """
        self.robot = 1
        self.goals = []
        self.ts = ts
        self.buchi_graph = buchi_graph
        self.init = init
        self.step_size = step_size
        self.dim = len(self.ts['workspace'])
        uni_v = np.power(np.pi, self.robot*self.dim/2) / math.gamma(self.robot*self.dim/2+1)
        self.gamma = np.ceil(4 * np.power(1/uni_v, 1./(self.dim*self.robot)))   # unit workspace
        self.tree = DiGraph(type='PBA', init=init)
        label = self.label(init[0])
        if label != '':
            label = label + '_' + str(1)
        # accepting state before current node
        acc = set()
        if 'accept' in init[1]:
            acc.add(init)
        self.tree.add_node(init, cost=0, label=label, acc=acc)
        self.search_goal(init, label, acc)

        # already used skilles
        self.used = set()
        self.base = base

    def sample(self):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """
        x_rand = []
        for i in range(self.dim):
            x_rand.append(uniform(0, self.ts['workspace'][i]))

        return tuple(x_rand)

    def nearest(self, x_rand):
        """
        find the nearest class of vertices in the tree
        :param: x_rand randomly sampled point form: single point ()
        :return: nearest class of vertices form: single point ()
        """
        min_dis = math.inf
        q_nearest = []
        for vertex in self.tree.nodes:
            x_vertex = vertex[0]
            dis = np.linalg.norm(np.subtract(x_rand, x_vertex))
            if dis < min_dis:
                q_nearest = list()
                q_nearest.append(vertex)
                min_dis = dis
            elif dis == min_dis:
                q_nearest.append(vertex)
        return q_nearest

    def steer(self, x_rand, x_nearest):
        """
        steer
        :param: x_rand randomly sampled point form: single point ()
        :param: x_nearest nearest point in the tree form: single point ()
        :return: new point single point ()
        """
        if np.linalg.norm(np.subtract(x_rand, x_nearest)) <= self.step_size:
            return x_rand
        else:
            return tuple(np.asarray(x_nearest) + self.step_size *
                         (np.subtract(x_rand, x_nearest))/np.linalg.norm(np.subtract(x_rand, x_nearest)))

    def acpt_check(self, q_min, q_new):
        """
        check the accepting state in the patg leading to q_new
        :param q_min:
        :param q_new:
        :return:
        """
        changed = False
        acc = set(self.tree.nodes[q_min]['acc']) # copy
        if 'accept' in q_new[1]:
            acc.add(q_new)
            # print(acc)
            changed = True
        return acc, changed

    def search_goal(self, q_new, label_new, acc):
        """
        whether q_new can connect to point before acc
        :param q_new:
        :param label_new:
        :param acc:
        :return:
        """
        for ac in acc:
            # connect to path leading to accepting state, including accepting state
            path = self.findpath([ac])[0][1]
            for point in path:
                if list(self.obs_check([q_new], point[0], self.tree.nodes[point]['label']).values())[0] \
                        and self.checkTranB(q_new[1], label_new, point[1]):
                    self.goals.append((q_new, point, ac))  # endpoint, middle point, accepting point

    def extend(self, q_new, near_v, label, obs_check, succ_list):
        """
        :param: q_new: new state form: tuple (mulp, buchi)
        :param: near_v: near state form: tuple (mulp, buchi)
        :param: obs_check: check obstacle free  form: dict { (mulp, mulp): True }
        :param: succ: list of successor of the root
        :return: extending the tree
        """
        added = 0
        cost = np.inf
        q_min = ()
        for near_vertex in near_v:
            if near_vertex in succ_list:
                continue
            if q_new != near_vertex and obs_check[(q_new[0], near_vertex[0])] \
                    and self.checkTranB(near_vertex[1], self.tree.nodes[near_vertex]['label'], q_new[1]):
                c = self.tree.nodes[near_vertex]['cost'] + \
                    np.linalg.norm(np.subtract(q_new[0], near_vertex[0]))
                if c < cost:
                    added = 1
                    q_min = near_vertex
                    cost = c
        if added == 1:
            self.tree.add_node(q_new, cost=cost, label=label)
            self.tree.nodes[q_new]['acc'] = set(self.acpt_check(q_min, q_new)[0])
            self.tree.add_edge(q_min, q_new)
            # self.search_goal(q_new, label, self.tree.nodes[q_new]['acc'])
        return added

    def rewire(self, q_new, near_v, obs_check):
        """
        :param: q_new: new state form: tuple (mul, buchi)
        :param: near_v: near state form: tuple (mul, buchi)
        :param: obs_check: check obstacle free form: dict { (mulp, mulp): True }
        :return: rewiring the tree
        """
        for near_vertex in near_v:
            if obs_check[(q_new[0], near_vertex[0])] \
                    and self.checkTranB(q_new[1], self.tree.nodes[q_new]['label'], near_vertex[1]):
                c = self.tree.nodes[q_new]['cost'] \
                    + np.linalg.norm(np.subtract(q_new[0], near_vertex[0]))
                delta_c = self.tree.nodes[near_vertex]['cost'] - c
                # update the cost of node in the subtree rooted at near_vertex
                if delta_c > 0:
                    # self.tree.nodes[near_vertex]['cost'] = c
                    self.tree.remove_edge(list(self.tree.pred[near_vertex].keys())[0], near_vertex)
                    self.tree.add_edge(q_new, near_vertex)
                    edges = dfs_labeled_edges(self.tree, source=near_vertex)
                    acc, changed = self.acpt_check(q_new, near_vertex)
                    self.tree.nodes[near_vertex]['acc'] = set(acc)
                    for u, v, d in edges:
                        if d == 'forward':
                            self.tree.nodes[v]['cost'] = self.tree.nodes[v]['cost'] - delta_c
                            if changed:
                                self.tree.nodes[v]['acc'] = set(self.acpt_check(u, v)[0])  # copy
        # better to research the goal but abandon the implementation

    def near(self, x_new):
        """
        find the states in the near ball
        :param x_new: new point form: single point
        :return: p_near: near state, form: tuple (mulp, buchi)
        """
        p_near = []
        r = min(self.gamma * np.power(np.log(self.tree.number_of_nodes()+1)/self.tree.number_of_nodes(),
                                      1./(self.dim*self.robot)), self.step_size)
        # r = self.step_size
        for vertex in self.tree.nodes:
            if np.linalg.norm(np.subtract(x_new, vertex[0])) <= r:
                p_near.append(vertex)
        return p_near

    def obs_check(self, q_near, x_new, label):
        """
        check whether obstacle free along the line from x_near to x_new
        :param q_near: states in the near ball, tuple (mulp, buchi)
        :param x_new: new state form: multiple point
        :param label: label of x_new
        :param stage: regular stage or final stage, deciding whether it's goal state
        :return: dict (x_near, x_new): true (obs_free)
        """

        obs_check_dict = {}
        checked = set()

        for x in q_near:
            if x[0] in checked:
                continue
            checked.add(x[0])
            obs_check_dict[(x_new, x[0])] = True

            # the line connecting two points crosses an obstacle
            for (obs, boundary) in iter(self.ts['obs'].items()):
                if LineString([Point(x[0]), Point(x_new)]).intersects(boundary):
                    obs_check_dict[(x_new, x[0])] = False
                    break

            for (region, boundary) in iter(self.ts['region'].items()):
                if LineString([Point(x[0]), Point(x_new)]).intersects(boundary) \
                        and region + '_' + str(1) != label \
                        and region + '_' + str(1) != self.tree.nodes[x]['label']:
                    # if stage == 'reg' or (stage == 'final' and region in self.no):
                    obs_check_dict[(x_new, x[0])] = False
                    break

        return obs_check_dict

    def label(self, x):
        """
        generating the label of position state
        :param x: position
        :return: label
        """

        point = Point(x)
        # whether x lies within obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if point.within(boundary):
                return obs

        # whether x lies within regions
        for (region, boundary) in iter(self.ts['region'].items()):
            if point.within(boundary):
                return region
        # x lies within unlabeled region
        return ''

    def checkTranB(self, b_state, x_label, q_b_new):
        """ decide valid transition, whether b_state --L(x)---> q_b_new
             Algorithm2 in Chapter 2 Motion and Task Planning
             :param b_state: buchi state
             :param x_label: label of x
             :param q_b_new buchi state
             :return True satisfied
        """
        b_state_succ = self.buchi_graph.succ[b_state]
        # q_b_new is not the successor of b_state
        if q_b_new not in b_state_succ:
            return False

        truth = self.buchi_graph.edges[(b_state, q_b_new)]['truth']
        if self.t_satisfy_b_truth(x_label, truth):
            return True

    def t_satisfy_b_truth(self, x_label, truth):
        """
        check whether transition enabled under current label
        :param x_label: current label
        :param truth: truth value making transition enabled
        :return: true or false
        """
        if truth == '1':
            return True

        true_label = [truelabel for truelabel in truth.keys() if truth[truelabel]]
        for label in true_label:
            if label not in x_label:
                return False

        false_label = [falselabel for falselabel in truth.keys() if not truth[falselabel]]
        for label in false_label:
            if label in x_label:
                return False

        return True

    def findpath(self, goals):
        """
        find the path backwards
        :param goals: goal state
        :return: dict path : cost
        """
        paths = OrderedDict()
        for i in range(len(goals)):
            goal = goals[i]
            path = [goal]
            s = goal
            while s != self.init:
                s = list(self.tree.pred[s].keys())[0]
                if s == path[0]:
                    print("loop")
                path.insert(0, s)

            paths[i] = [self.tree.nodes[goal]['cost'], path]
        return paths


# def extend_rewire(subtree, x_new, near_v, label, buchi_graph):
#
#     # check obstacle free
#     obs_check = subtree.obs_check(near_v, x_new, label, 'reg')
#
#     # iterate over each buchi state
#     for b_state in buchi_graph.nodes:
#
#         # new product state
#         q_new = (x_new, b_state)
#
#         # extend
#         added = subtree.extend(q_new, near_v, label, obs_check)
#         # rewire
#         if added == 1:
#             subtree.rewire(q_new, near_v, obs_check)
#
#     # number of nodes
#     # sz.append(tree.tree.number_of_nodes())


def construction_tree(subtree, x_rand, buchi_graph, centers, h_task, flag, connect):
    # # sample
    # x_rand = subtree.sample()
    # nearest
    q_nearest = subtree.nearest(x_rand)
    # steer
    x_new = subtree.steer(x_rand, q_nearest[0][0])
    # label
    label = subtree.label(x_new)
    if 'o' in label:
        return
    if label != '':
        label = label + '_' + str(1)

    # near state
    near_v = subtree.near(x_new)
    # add q_rand
    if q_nearest[0] not in near_v:
        near_v = near_v + q_nearest
    # extend and rewire

    # check obstacle free
    obs_check = subtree.obs_check(near_v, x_new, label)

    # successor root of current root
    curr = None
    for v in h_task.nodes:
        if v.x == subtree.init[0] and v.q == subtree.init[1]:
            curr = v
            break
    succ = [sc.xq() for sc in h_task.succ[curr]]
    # iterate over each buchi state
    for b_state in buchi_graph.nodes():

        # new product state
        q_new = (x_new, b_state)

        # extend
        added = subtree.extend(q_new, near_v, label, obs_check, succ)
        # rewire
        if added:
            subtree.rewire(q_new, near_v, obs_check)
            if flag:
                construction_tree_connect_root(subtree, q_new, label, centers, h_task, connect)


def construction_tree_connect_root(subtree, q_new, label, centers, h_task, connect):
    # extend towards to other roots

    curr = None
    for v in h_task.nodes:
        if v.x == subtree.tree.graph['init'][0] and v.q == subtree.tree.graph['init'][1]:
            curr = v
            break

    for sc in h_task.succ[curr]:
        if sc == curr:
            continue
        succ = sc.xq()
        # label of succ
        label_succ = None
        for l, coord in centers.items():
            if coord == succ[0]:
                label_succ = l + '_' + str(1)
                break
        # connect q_new to succ
        if list(subtree.obs_check([q_new], succ[0], label_succ).values())[0] and subtree.checkTranB(q_new[1], label, succ[1]):
            c = subtree.tree.nodes[q_new]['cost'] + \
                            np.linalg.norm(np.subtract(q_new[0], succ[0]))
            # in the tree
            if succ in subtree.tree.nodes:
                delta_c = subtree.tree.nodes[succ]['cost'] - c
                # update the cost of node in the subtree rooted at near_vertex
                if delta_c > 0:
                    subtree.tree.remove_edge(list(subtree.tree.pred[succ].keys())[0], succ)
                    subtree.tree.nodes[succ]['acc'] = set(subtree.acpt_check(q_new, succ)[0])
                    subtree.tree.add_edge(q_new, succ)

            # not in the tree
            else:
                subtree.tree.add_node(succ, cost=c, label=label_succ, acc=set(subtree.acpt_check(q_new, succ)[0]))
                subtree.tree.add_edge(q_new, succ)
                # keep track of connection
                connect.add((curr, sc))


def multi_trees(h_task, buchi_graph, ts, centers, max_node):
    multi_tree = list()
    # a list of subtrees
    for root in h_task.nodes():
        init = root.xq()
        multi_tree.append(tree(ts, buchi_graph, init, 0.25))
    # =====================================================================================
    # n_max = n_max
    # c = 0
    # connect = set()
    # # for n in range(n_max):
    # now = datetime.now()
    # while np.sum([t.tree.number_of_nodes() for t in multi_tree]) < max_node:
    #     # print(n)
    #     i = random.randint(0, len(multi_tree)-1)
    #     # sample
    #     x_rand = multi_tree[i].sample()
    #     # if n <= c * n_max:
    #     # construction_tree(multi_tree[i], x_rand, buchi_graph, centers, h_task, 0, connect)
    #     # else:
    #     construction_tree(multi_tree[i], x_rand, buchi_graph, centers, h_task, 1, connect)
    #
    # end2path = dict()
    # for pair in connect:
    #     for t in range(len(multi_tree)):
    #         if multi_tree[t].tree.graph['init'] == (pair[0].x, pair[0].q):
    #             end2path[pair] = multi_tree[t].findpath([(pair[1].x, pair[1].q)])[0][1]
    #             break
    # num_path_seq = len(end2path.keys())
    # time1 = (datetime.now() - now).total_seconds()
    #
    # # option 2 parallel
    # multi_tree = list()
    # # a list of subtrees
    # for root in h_task.nodes:
    #     if 'accept' not in root.q:
    #         init = (root.x, root.q)
    #         multi_tree.append(tree(ts, buchi_graph, init, 0.25, no))
    #
    # # ====================================================================================
    c = 0
    connect = set()
    # for n in range(n_max):
    now = datetime.now()
    while np.sum([t.tree.number_of_nodes() for t in multi_tree]) < max_node:
        # print(n)
        x_rand = multi_tree[0].sample()

        for i in range(len(multi_tree)):
            # if n <= c * n_max:
            if np.sum([t.tree.number_of_nodes() for t in multi_tree]) < c * max_node:  # 0 is better
                construction_tree(multi_tree[i], x_rand, buchi_graph, centers, h_task, 0, connect)
            else:
                construction_tree(multi_tree[i], x_rand, buchi_graph, centers, h_task, 1, connect)

    end2path = dict()
    for pair in connect:
        for t in range(len(multi_tree)):
            if multi_tree[t].tree.graph['init'] == pair[0].xq():
                end2path[pair] = multi_tree[t].findpath([pair[1].xq()])[0][1]
                break

    time2 = (datetime.now() - now).total_seconds()
    print(time2, h_task.number_of_edges(), len(end2path.keys()))
    # print(time1, time2, num_path_seq, num_path_par)
    return end2path
