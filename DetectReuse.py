from sympy import satisfiable
from networkx.classes.digraph import DiGraph
from sympy.logic.boolalg import to_dnf
from sympy.logic.boolalg import Not, And, Equivalent
from state import State
from DetermineRoots import target


# def find_node(cand, h_task):
#     """
#
#     :param cand:
#     :param h_task:
#     :return:
#     """
#     for node in h_task.nodes:
#         if cand == node:
#             return node
#     return cand


def hoftask_no_simplified(init, buchi_graph, regions):
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
        for q_b in buchi_graph.succ[curr.q]:
            try:
                label = to_dnf(buchi_graph.edges[(q_b, q_b)]['label'], True)
            except KeyError:
                # no self loop
                label = to_dnf('0')
            if q_b != curr.q:
                # region corresponding to the label/word
                edge_label = (buchi_graph.edges[(curr.q, q_b)]['label'])
                if edge_label == '(1)':
                    x_set = [curr.x]
                else:
                    x_set = target(to_dnf(edge_label), regions)
                if x_set:
                    for x in x_set:
                        cand = State((x, q_b), label)
                        #  whether cand is already in the h_task
                        # cand = find_node(cand, h_task)
                        h_task.add_edge(curr, cand)
                        if cand not in open_set and cand not in explore_set:
                            open_set.append(cand)

        explore_set.append(curr)
    # for x in h_task.nodes:
    #     print(x)
    return h_task


def inclusion(subtask_lib, subtask_new):
    """
    whether subtask_lib is included in subtask_new
    :param subtask_lib:
    :param subtask_new:
    :return:
    """
    # if subtask_new[0].q == 'T2_S3':
    #     print('hi')
    # if Equivalent(subtask_lib[0].label, subtask_new[0].label):
    #     print(subtask_lib[0].label, subtask_new[0].label)
    # A included in B <=> does no exist a truth assignment such that  (A & not B) is true
    if not satisfiable(And(subtask_lib[0].label, Not(subtask_new[0].label))):
        if subtask_lib[0].x == subtask_new[0].x and subtask_lib[1].x == subtask_new[1].x:
            return 'forward'
        elif subtask_lib[0].x == subtask_new[1].x and subtask_lib[1].x == subtask_new[0].x:
            return 'backward'

    return ''


def replace(init_q, end_q, init_lib_q, end_lib_q, path, direction):
    """
    replace buchi states in the original lib subtask with new buchi states
    :param init_q:
    :param end_q:
    :param init_lib_q:
    :param end_lib_q:
    :param path:
    :param direction:
    :return:
    """

    assert init_lib_q == path[0][1], 'match error'
    assert end_lib_q == path[-1][1], 'match error'

    repath = []
    if direction == 'f':
        for point in path[:-1]:
            repath.append((point[0], init_q))
        repath.append((path[-1][0], end_q))
    elif direction == 'b':
        # reverse
        # initial state
        p = path[::-1]
        repath.append((p[0][0], init_q))
        # intermediate state with init_Q
        if len(p) > 2:
            for point in p[1:-1]:
                repath.append((point[0], init_q))
        # end state
        repath.append((p[-1][0], end_q))
    return repath


def detect_reuse(h_task_lib, h_task_new, end2path):
    """
    detect resuable subtask
    :param h_task_lib:
    :param h_task_new:
    :param end2path:
    :return:
    """
    subtask2path = dict()
    # each edge in new h_task
    for subtask_new in h_task_new.edges:
        # match edges in library of h_task
        for subtask_lib in h_task_lib.edges:
            # future work: it's better to compare the lib of controller
            if subtask_lib not in end2path.keys():
                continue
            direction = inclusion(subtask_lib, subtask_new)
            if direction == 'forward':
                subtask2path[((subtask_new[0].x, subtask_new[0].q), (subtask_new[1].x, subtask_new[1].q))] \
                        = replace(subtask_new[0].q, subtask_new[1].q, subtask_lib[0].q, subtask_lib[1].q, end2path[subtask_lib], 'f')
                break
            elif direction == 'backward':
                subtask2path[((subtask_new[0].x, subtask_new[0].q), (subtask_new[1].x, subtask_new[1].q))] \
                    = replace(subtask_new[0].q, subtask_new[1].q, subtask_lib[0].q, subtask_lib[1].q, end2path[subtask_lib], 'b')
                break

    # starting point : subtask starting from starting point
    starting2waypoint = {key[0]: [] for key, _ in subtask2path.items()}
    for key, _ in subtask2path.items():
        starting2waypoint[key[0]].append(key)
    # including accepting state
    # for node in h_task_new.nodes:
    #     if 'accept' in node.q and (node.x, node.q) not in starting2waypoint.keys():
    #         starting2waypoint[(node.x, node.q)] = []
    return subtask2path, starting2waypoint
