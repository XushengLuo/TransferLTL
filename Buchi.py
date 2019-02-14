# -*- coding: utf-8 -*-

import subprocess
import os.path
import re
import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
import pyvisgraph as vg
import itertools
from sympy.logic.boolalg import to_cnf, Or
from sympy import satisfiable


class buchi_graph(object):
    """ construct buchi automaton graph
    Parameter:
        formula: LTL formula specifying task
    """

    def __init__(self, formula, formula_comp, exclusion):
        self.formula = formula
        self.formula_comp = formula_comp
        self.exclusion = exclusion

    def formulaParser(self):
        """replace letter with symbol
        """
        indicator = 'FG'

        if [True for i in indicator if i in self.formula]:
            self.formula.replace('F', '<>').replace('G', '[]')

    def execLtl2ba(self):
        """ given formula, exectute the ltl2ba
        Parameter:
            buchi_str: output string of program ltl2ba  (utf-8 format)
        """

        dirname = os.path.dirname(__file__)
        self.buchi_str = subprocess.check_output(dirname + "/./ltl2ba -f \"" + self.formula + "\"", shell=True).decode("utf-8")
        # print(self.buchi_str)


    def buchiGraph(self):
        """parse the output of ltl2ba
        Parameter:
            buchi_graph: Graph of buchi automaton
        """
        # find all states
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, self.buchi_str)

        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        accep = [s for s in state_group if 'accept' in s]

        """
        Format:
            buchi_graph.node = NodeView(('T0_init', 'T1_S1', 'accept_S1'))
            buchi_graph.edges = OutEdgeView([('T0_init', 'T0_init'), ('T0_init', 'T1_S1'),....])
            buchi_graph.succ = AdjacencyView({'T0_init': {'T0_init': {'label': '1'}, 'T1_S1': {'label': 'r3'}}})
        """
        self.buchi_graph = DiGraph(type='buchi', init=init, accept=accep)
        order_key = list(self.formula_comp.keys())
        order_key.sort(reverse=True)
        for state in state_group:
            # for each state, find transition relation
            # add node
            self.buchi_graph.add_node(state)
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', self.buchi_str, re.DOTALL)
            if state_if_fi:
                relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
                for (labell, state_dest) in relation_group:
                    # whether the edge is feasible in terms of unit atomic proposition
                    label = self.InitialDelInfesEdge(labell)
                    if not label or label.isspace():
                        continue
                    # add edge
                    for k in order_key:
                        if k >= 10:
                            label = label.replace('e_{0}'.format(k), self.formula_comp[k])
                        else:
                            label = label.replace('e{0}'.format(k), self.formula_comp[k])
                    # if '!' in label:
                    #     label = self.PutNotInside(label)
                    label = label.replace('||', '|').replace('&&', '&').replace('!', '~')
                    self.buchi_graph.add_edge(state, state_dest, label=label)

        return self.buchi_graph

    def ShorestPathBtRg(self, regions):
        """
        calculate shoresr path between any two labeled regions
        :param regions: regions
        :return: dict (region, region) : length
        """
        polys = [[vg.Point(0.4, 1.0), vg.Point(0.4, 0.7), vg.Point(0.6, 0.7), vg.Point(0.6, 1.0)],
                 [vg.Point(0.3, 0.2), vg.Point(0.3, 0.0), vg.Point(0.7, 0.0), vg.Point(0.7, 0.2)]]
        g = vg.VisGraph()
        g.build(polys, status=False)

        min_len_region = dict()
        for key1, value1 in regions.items():
            for key2, value2 in regions.items():
                init = value1[:2]
                tg = value2[:2]
                # shorest path between init and tg point
                shortest = g.shortest_path(vg.Point(init[0], init[1]), vg.Point(tg[0], tg[1]))
                # (key2, key1) is already checked
                if (key2, key1) in min_len_region.keys():
                    min_len_region[(key1, key2)] = min_len_region[(key2, key1)]
                else:
                    # different regions
                    if key1 != key2:
                        dis = 0
                        for i in range(len(shortest)-1):
                            dis = dis + np.linalg.norm(np.subtract((shortest[i].x, shortest[i].y), (shortest[i+1].x, shortest[i+1].y)))

                        min_len_region[(key1, key2)] = dis
                    # same region
                    else:
                        min_len_region[(key1, key2)] = 0

        return min_len_region

    def RobotRegion(self, exp, robot):
        """
        pair of robot and corresponding regions in the expression
        :param exp: logical expression
        :param robot: # of robots
        :return: dic of robot index : regions
        exp = 'l1_1 & l3_1 & l4_1 & l4_6 | l3_4 & l5_6'
        {1: ['l1_1', 'l3_1', 'l4_1'], 4: ['l3_4'], 6: ['l4_6', 'l5_6']}
        """

        robot_region_dict = dict()
        for r in range(robot):
            findall = re.findall(r'(l\d+?_{0})[^0-9]'.format(r + 1), exp)
            if findall:
                robot_region_dict[str(r + 1)] = findall

        return robot_region_dict

    def FeasTruthTable(self, exp, robot_region):
        """
        Find feasible truth table to make exp true
        :param exp: expression
        :return:
        """
        if exp == '(1)':
            return '1'

        sgl_value = []
        for key, value in robot_region.items():
            if len(value) == 1:
                sgl_value.append(value[0])

        # set all to be false
        exp1 = to_cnf(exp)
        value_in_exp = [value.name for value in exp1.atoms()]
        subs = {true_rb_rg: False for true_rb_rg in value_in_exp}
        if exp1.subs(subs):
            return subs

        # set one to be true, the other to be false
        for prod in itertools.product(*robot_region.values()):
            exp1 = exp
            # set one specific item to be true
            for true_rb_rg in prod:
                # set the other to be false
                value_cp = list(robot_region[true_rb_rg.split('_')[1]])
                if len(value_cp) > 1:
                    value_cp.remove(true_rb_rg)
                    # replace the rest with same robot to be ~
                    for v_remove in value_cp:
                        exp1 = exp1.replace(v_remove, '~' + true_rb_rg)

            # simplify
            exp1 = to_cnf(exp1)
            # all value in expression
            value_in_exp = [value.name for value in exp1.atoms()]
            # all single value in expression
            sgl_value_in_exp = [value for value in value_in_exp if value in sgl_value]
            # not signle value in expression
            not_sgl_value_in_exp = [value for value in value_in_exp if value not in sgl_value]

            subs1 = {true_rb_rg: True for true_rb_rg in not_sgl_value_in_exp}

            tf = [False, True]
            # if type(exp1) == Or:
            #     tf = [False, True]

            if len(sgl_value_in_exp):
                for p in itertools.product(*[tf] * len(sgl_value_in_exp)):
                    subs2 = {sgl_value_in_exp[i]: p[i] for i in range(len(sgl_value_in_exp))}
                    subs = {**subs1, **subs2}
                    if exp1.subs(subs):
                        return subs
            else:
                if exp1.subs(subs1):
                    return subs1

        return []

    def DelInfesEdge(self, robot):
        """
        Delete infeasible edge
        :param buchi_graph: buchi automaton
        :param robot: # robot
        """
        TobeDel = []
        # print(self.buchi_graph.number_of_edges())
        i = 0
        for edge in self.buchi_graph.edges():
            i = i+1
            # print(i)
            b_label = self.buchi_graph.edges[edge]['label']
            if b_label != '(1)':
                exp = b_label.replace('||', '|').replace('&&', '&').replace('!', '~')
                truth = satisfiable(exp, algorithm="dpll")
                truth_table = dict()
                for key, value in truth.items():
                    truth_table[key.name] = value
                if not truth_table:
                    TobeDel.append(edge)
                else:
                    self.buchi_graph.edges[edge]['truth'] = truth_table
            else:
                self.buchi_graph.edges[edge]['truth'] = '1'

        for edge in TobeDel:
            self.buchi_graph.remove_edge(edge[0], edge[1])
        # print(self.buchi_graph.number_of_edges())

    def InitialDelInfesEdge(self, orig_label):
        div_by_or = orig_label.split(') || (')

        for item in div_by_or:
            feas = True
            for excl in self.exclusion:
                # mutual exclusion term exist
                if excl[0] in item and excl[1] in item and '!{0}'.format(excl[0]) not in item and '!{0}'.format(excl[1]) not in item:
                    feas = False
                    break
            if not feas:
                item = item.strip('(').strip(')')
                item = '(' + item + ')'
                orig_label = orig_label.replace(' '+item+' ||', '').replace(item+' || ','').replace(' || '+item,'').replace(item,'')

        return orig_label

    def MinLen(self):
        """
        search the shorest path from a node to another, weight = 1, i.e. # of state in the path
        :param buchi_graph:
        :return: dict of pairs of node : length of path
        """
        min_qb_dict = dict()
        for node1 in self.buchi_graph.nodes():
            for node2 in self.buchi_graph.nodes():
                if node1 != node2 and 'accept' in node2:
                    try:
                        l, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph, source=node1, target=node2)
                    except nx.exception.NetworkXNoPath:
                        l = np.inf
                        # path = []
                    min_qb_dict[(node1, node2)] = l
                elif node1 == node2 and 'accept' in node2:
                    l = np.inf
                    # path = []
                    for succ in self.buchi_graph.succ[node1]:
                        try:
                            l0, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph, source=succ, target=node1)
                        except nx.exception.NetworkXNoPath:
                            l0 = np.inf
                            # path0 = []
                        if l0 < l:
                            l = l0 + 1
                            # path = path0
                    min_qb_dict[(node1, node2)] = l

        return min_qb_dict

    def FeasAcpt(self, min_qb):
        """
        delte infeasible final state
        :param buchi_graph: buchi automaton
        :param min_qb: dict of pairs of node : length of path
        """
        accept = self.buchi_graph.graph['accept']
        for acpt in accept:
            if min_qb[(self.buchi_graph.graph['init'][0], acpt)] == np.inf or min_qb[(acpt, acpt)] == np.inf:
                self.buchi_graph.graph['accept'].remove(acpt)

    def PutNotInside(self, str):
        """
        put not inside the parenthesis !(p1 && p2) -> !p1 or !p2
        :param str: old
        :return: new
        """
        substr = re.findall("(!\(.*?\))", str)  # ['!(p1 && p2)', '!(p4 && p5)']
        for s in substr:
            oldstr = s.strip().strip('!').strip('(').strip(')')
            nstr = ''
            for ss in oldstr.split():
                if '&&' in ss:
                    nstr = nstr + ' or '
                elif 'or' in ss:
                    nstr = nstr + ' && '
                else:
                    nstr = nstr + '!' + ss
            str = str.replace(s, nstr)
        return str

    def label2sat(self):
        for edge in self.buchi_graph.edges():
            label = self.buchi_graph.edges[edge]['label']
            label = label.replace('||', '|').replace('&&', '&').replace('!', '~')
            exp1 = to_cnf(label)
            self.buchi_graph.edges[edge]['label'] = exp1