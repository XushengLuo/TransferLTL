"""
__author__ = chrislaw
__project__ = TransferLTL
__date__ = 2/1/19
"""

from sympy import satisfiable
from networkx.classes.digraph import DiGraph
from sympy.logic.boolalg import to_dnf
from sympy.logic.boolalg import Equivalent, Not, And
from state import State


def target(exp, regions):
    """
    find the target location
    :param label: word
    :return: target location/region
    """

    target = []

    if exp != to_dnf('1'):
        # a set of target points
        for dnf in exp.args or [exp]:
            truth_table = satisfiable(dnf, algorithm="dpll")
            # find the target point with true value
            for key, value in truth_table.items():
                if value:
                    des = key.name.split('_')[0]
                    target.append(regions[des])
        return target


def match(h_task, curr, cand):
    """

    :param h_task:
    :param curr: parent node
    :param cand: child node
    :return: yes if there exists the same subtask
    """
    if curr.q == 'T0_S4':
        print('')
    for edge in h_task.edges:
        if (edge[0].x == curr.x and edge[1].x == cand.x) or (edge[1].x == curr.x and edge[0].x == cand.x) \
                and Equivalent(edge[0].label, curr.label):
            return True
    return False


def hoftask(init, buchi_graph, regions):
    h_task = DiGraph(type='subtask')
    try:
        label = to_dnf(buchi_graph.edges[(buchi_graph.graph['init'][0], buchi_graph.graph['init'][0])]['label'], True)
    except KeyError:
        # no self loop
        label = to_dnf('0')
    init_node = State((init, buchi_graph.graph['init'][0]), label)
    h_task.add_node(init_node)
    open_set = list()
    open_set.append(init_node)
    explore_set = list()

    while open_set:
        curr = open_set.pop(0)

        # assume co-safe
        # if curr.q in buchi_graph.graph['accept']:
        #     continue
        # print(curr)
        for q_b in buchi_graph.succ[curr.q]:

            try:
                label = to_dnf(buchi_graph.edges[(q_b, q_b)]['label'], True)
            except KeyError:
                # no self loop
                label = to_dnf('0')
            if q_b != curr.q: # and q_b not in buchi_graph.graph['accept']:
                # region corresponding to the label/word
                edge_label = (buchi_graph.edges[(curr.q, q_b)]['label'])
                if edge_label == '(1)':
                    x_set = [curr.x]
                else:
                    x_set = target(to_dnf(edge_label), regions)
                if x_set:
                    for x in x_set:
                        cand = State((x, q_b), label)
                        # print(cand)
                        if cand not in open_set and cand not in explore_set:
                            open_set.append(cand)
                            # check the match with each subtask in h_task
                            if not match(h_task, curr, cand):
                                # print(cand)
                                h_task.add_node(cand)
                                h_task.add_edge(curr, cand)
                                # continue
                            # roots for accepting states
                            # if q_b in buchi_graph.graph['accept']:
                            #     h_task.add_edge(curr, cand)

        explore_set.append(curr)
    # for x in h_task.nodes:
    #     print(x)
    return h_task



