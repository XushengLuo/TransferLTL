"""
__author__ = chrislaw
__project__ = TransferLTL
__date__ = 2/1/19
"""

from shapely.geometry import Point, Polygon
import sys


class problemFormulation(object):
    def __init__(self, case):
        # +----------------------------------------------+
        # |                                              |
        # |                 Problem 1                    |
        # |                                              |
        # +----------------------------------------------+


        # +-----+-----+-----+
        # |  l1 |     | l2  |
        # |     +-----+     |
        # |       l4        |
        # |             l3  |
        # |    +-------+    |
        # | l5 |       |    |
        # +----+-------+----+
        # l1: (0.2, 0.8)
        # l2: (0.8, 0.8)
        # l3: (0.8, 0.4)
        # l4: (0.4, 0.4)
        # l5: (0.1, 0.2)

        self.workspace = (1, 1)
        # !! no whitespace in atomic proposation      b:ball s:square
        r = 0.05  # float(sys.argv[2])
        self.ap = {'l1', 'l2', 'l3', 'l4', 'l5', 'l6'}

        center = [(0.2, 0.8), (0.8, 0.8), (0.8, 0.4), (0.4, 0.4), (0.1, 0.2), (0.1, 0.5)]

        self.centers = {'l1': center[0],
                        'l2': center[1],
                        'l3': center[2],
                        'l4': center[3],
                        'l5': center[4],
                        'l6': center[5],
                        }

        self.regions = {'l1': Polygon([(center[0][0] - r, center[0][1] - r), (center[0][0] + r, center[0][1] - r),
                                       (center[0][0] + r, center[0][1] + r), (center[0][0] - r, center[0][1] + r)]),
                        'l2': Polygon([(center[1][0] - r, center[1][1] - r), (center[1][0] + r, center[1][1] - r),
                                       (center[1][0] + r, center[1][1] + r), (center[1][0] - r, center[1][1] + r)]),
                        'l3': Polygon([(center[2][0] - r, center[2][1] - r), (center[2][0] + r, center[2][1] - r),
                                       (center[2][0] + r, center[2][1] + r), (center[2][0] - r, center[2][1] + r)]),
                        'l4': Polygon([(center[3][0] - r, center[3][1] - r), (center[3][0] + r, center[3][1] - r),
                                       (center[3][0] + r, center[3][1] + r), (center[3][0] - r, center[3][1] + r)]),
                        'l5': Polygon([(center[4][0] - r, center[4][1] - r), (center[4][0] + r, center[4][1] - r),
                                       (center[4][0] + r, center[4][1] + r), (center[4][0] - r, center[4][1] + r)]),
                        'l6': Polygon([(center[5][0] - r, center[5][1] - r), (center[5][0] + r, center[5][1] - r),
                                       (center[5][0] + r, center[5][1] + r), (center[5][0] - r, center[5][1] + r)])
                        }

        center = [(0.6, 0.3), (0.6, 0.5), (0.3, 0.6), (0.8, 0.6), (0.2, 0.3)]
        self.obs = {'o1': Polygon([(0.3, 0.0), (0.7, 0.0), (0.7, 0.2), (0.3, 0.2)]),
                    'o2': Polygon([(0.4, 0.7), (0.6, 0.7), (0.6, 1.0), (0.4, 1.0)]),
                    'o3': Polygon([(center[0][0] - r, center[0][1] - r), (center[0][0] + r, center[0][1] - r),
                                   (center[0][0] + r, center[0][1] + r), (center[0][0] - r, center[0][1] + r)]),
                    'o4': Polygon([(center[1][0] - r, center[1][1] - r), (center[1][0] + r, center[1][1] - r),
                                   (center[1][0] + r, center[1][1] + r), (center[1][0] - r, center[1][1] + r)]),
                    'o5': Polygon([(center[2][0] - r, center[2][1] - r), (center[2][0] + r, center[2][1] - r),
                                   (center[2][0] + r, center[2][1] + r), (center[2][0] - r, center[2][1] + r)]),
                    'o6': Polygon([(center[3][0] - r, center[3][1] - r), (center[3][0] + r, center[3][1] - r),
                                   (center[3][0] + r, center[3][1] + r), (center[3][0] - r, center[3][1] + r)]),
                    'o7': Polygon([(center[4][0] - r, center[4][1] - r), (center[4][0] + r, center[4][1] - r),
                                   (center[4][0] + r, center[4][1] + r), (center[4][0] - r, center[4][1] + r)])
                    }

        init_state = []
        r = {0: 1,
             1: 1,
             2: 2,
             21: 4,
             3: 16,
             4: 16,
             5: 16,
             6: 20,
             7: 20}
        case = case #int(sys.argv[1])
        for i in range(r[case]):
            init_state.append((1, 0))
        self.init_state = tuple(init_state)
        # self.init_state = ((0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1))
        # self.init_state = ((0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1))
        # self.init_state = ((0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1))
        # self.init_state = ((0.8, 0.1), (0.9, 0.1))
        # self.init_state = ((0.8, 0.1), )
        self.uni_cost = 0.1

        # #----------------------------------------------#
        # |                                              |
        # |                 Problem 2                    |
        # |                                              |
        # #----------------------------------------------#

        # +-----+-----+-----+
        # | r4,b|r5,rb| r6  |
        # +-----+-----+-----+
        # | c1  | c2  | c3  |
        # +-----+-----+-----+
        # | r1  | r2,b|r3,gb|
        # +-----+-----+-----+



        """
        +----------------------------+
        |   Propositonal Symbols:    |
        |        true, false         |
        |	    any lowercase string |
        |                            |
        |   Boolean operators:       |
        |       !   (negation)       |
        |       ->  (implication)    |
        |	    <-> (equivalence)    |
        |       &&  (and)            |
        |       ||  (or)             |
        |                            |
        |   Temporal operators:      |
        |       []  (always)         |
        |       <>  (eventually)     |
        |       U   (until)          |
        |       V   (release)        |
        |       X   (next)           |
        +----------------------------+
        """

        # self.formula = '<>(l3 && []<>l4)'
        # self.formula = '<> (l11) &&   [](<> ( l21 && <> (l31 && <> l41 ) ) )'

        if case == 0:
            # self.formula = '<> e1 && <> e2'
            # self.formula = '<> e1 && !e1 U e2 '
            #
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                     }
            # self.exclusion = [('e1', 'e2')]
            # self.no = []

            # self.formula = '<> e1 && <> e2 && <> e3 && <> e5'
            self.formula = '<> e1 && <> e2 && <> e3'

            self.formula_comp = {1: '(l1_1)',
                                 2: '(l2_1)',
                                 3: '(l5_1)',
                                 4: '(l4_1)',
                                 5: '(l6_1)'}
            self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
                              ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5')]
            self.no = []

        # ---------------------------- case 1 --------------------------------
        if case == 1:
            # self.formula = '[]<> (e1 && <> (e2 && <> e3))'
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3')]
            # self.no = []
            # self.formula = '<> e1 && <> e2 && <> e3'
            #
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)',
            #                      4: '(l4_1)',
            #                      5: '(l6_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
            #                   ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5')]
            # self.no = []

            # self.formula = '<> e1 && <> e2 && []!e5'
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3')]
            # self.no = []
            #
            # self.formula = '<> e1 && <> e2 && <> e3 && <> e5'
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)',
            #                      4: '(l4_1)',
            #                      5: '(l6_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
            #                   ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5')]
            # self.no = []
            self.formula = '[]<> (e1 && <> e2) && []<> (e3 && <> e4)' #   && <> e5'  # && !e5 U e1 && !e5 U e3 && !e3 U e1'
            self.formula_comp = {1: '(l1_1)',
                                 2: '(l2_1)',
                                 3: '(l5_1)',
                                 4: '(l4_1)',
                                 5: '(l6_1)'}
            self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
                              ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5')]
            self.no = ['l6_1']

            # self.formula = '<> e1 && <> (e2  && <> (e3 && <> ( e4 && <> e5)))'
            #
            # self.formula_comp = {1: '(l4_1)',
            #                          2: '(l3_1)',
            #                          3: '(l1_1)',
            #                          4: '(l2_1)',
            #                          5: '(l5_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e1', 'e3'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'),
            #                       ('e2', 'e4'), ('e2', 'e5'), ('e3', 'e4'), ('e3', 'e5'), ('e4', 'e5')]
            # self.no = []

        # ---------------------------- case 1 --------------------------------
        # if case == 1:
        #     # self.formula = '<> l4_1 && []<> (l3_1 && <> l1_1) && (!l1_1 U l2_1)  && []!l5_1'
        #     self.formula = '<> e1 && []<> (e2  && <> e3) && (!e3 U e4)  && []!e5'
        #
        #     self.formula_comp = {1: '(l4_1)',
        #                          2: '(l3_1)',
        #                          3: '(l1_1)',
        #                          4: '(l2_1)',
        #                          5: '(l5_1)',
        #                          }
        #     self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e1', 'e3'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'),
        #                       ('e2', 'e4'), ('e2', 'e5'), ('e3', 'e4'), ('e3', 'e5'), ('e4', 'e5')]
        #     self.no = ['l5_1', 'l1_1']
        # #---------------------------- case 2 --------------------------------
        if case == 2:
            self.formula = '[]<> e1 && []<> e2 && []<> (e3 && <> e4)'
            self.formula_comp = {1: '(l1_1)',
                             2: '(l2_2)',
                             3: '(l4_1)',
                             4: '(l4_2)'}

            self.exclusion = []
            self.no = []

        # ---------------------------- case 2.1 --------------------------------
        # self.formula = '[]<> e1'
        # self.formula_comp = {1: '(l1_1 && l2_2 && l3_3 && l4_4 && l5_5 && l6_6)'}
        if case == 21:
            self.exclusion = []
            self.no = []
            self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6))'
            self.formula_comp = {
                1: '(l1_1 && l1_2)',
                2: '(l2_2 && l2_3)',
                3: '(l3_3 && l3_4)',
                4: '(l4_1)',
                5: '(l5_4)',
                6: '(l6_3)'}

        # # ---------------------------- case 3 --------------------------------
        if case == 3:
            self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6))'
            self.formula_comp = {
                1: '(l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)',
                2: '(l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)',
                3: '(l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)',
                4: '(l4_1)',
                5: '(l5_6)',
                6: '(l6_11)'}
            self.exclusion = [('e1', 'e2'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'), ('e2', 'e5'), ('e2', 'e6'),
                              ('e3', 'e6')]
            self.no = []

        # # ---------------------------- case 4 ----------------------------------
        #
        if case == 4:
            self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && []!e7'
            self.formula_comp = {
                1: '(l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)',
                2: '(l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)',
                3: '(l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)',
                4: '(l4_1)',
                5: '(l5_6)',
                6: '(l6_11)',
                7: '(l3_4 || l3_9)'}
            self.exclusion = [('e1', 'e2'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'), ('e2', 'e5'), ('e2', 'e6'),
                              ('e3', 'e6')]
            self.no = ['l3_4', 'l3_9']
        #
        # # ---------------------------- case 5 --------------------------------
        # # self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && []!e7 && [](e6 -> X(!e6 U e4)) && <> e8'
        if case == 5:
            self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && []!e7 && <> e8'
            self.formula_comp = {
        1: '(l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)',
        2: '(l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)',
        3: '(l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)',
        4: '(l4_1)',
        5: '(l5_6)',
        6: '(l6_11)',
        7: '(l3_4 || l3_9)',
        8: '(l4_3 && l5_8 && l6_13)'}
    # #
            self.exclusion = [('e1', 'e2'), ('e1', 'e4'), ('e1','e5'), ('e2','e3'), ('e2','e5'), ('e2','e6'), ('e3','e6')]
            self.no = ['l3_4','l3_9'] #, 'l6_7',
        #
        # # ---------------------------- case 6 --------------------------------
        if case == 6:
            self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && []!e7 && <> e8 && []<>e9'

            self.formula_comp = {
                1: '(l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)',
                2: '(l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)',
                3: '(l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)',
                4: '(l4_1)',
                5: '(l5_6)',
                6: '(l6_11)',
                7: '(l3_4 || l3_9)',
                8: '(l4_3 && l5_8 && l6_13)',
                9: '(l2_17 && l2_18 && l4_19 & l4_20)'}
            # #
            self.exclusion = [('e1', 'e2'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'), ('e2', 'e5'), ('e2', 'e6'),
                              ('e3', 'e6')]
            self.no = ['l3_4', 'l3_9']  # , 'l6_7',

        # ---------------------------- case 7 --------------------------------
        if case == 7:
           self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && []!e7 && <> e8 && []<>e9 && (!e8 U e9)'

           self.formula_comp = {
               1: '(l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)',
               2: '(l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)',
               3: '(l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)',
               4: '(l4_1)',
               5: '(l5_6)',
               6: '(l6_11)',
               7: '(l3_4 || l3_9)',
               8: '(l4_3 && l5_8 && l6_13)',
               9: '(l2_17 && l2_18 && l4_19 & l4_20)'}
           # #
           self.exclusion = [('e1', 'e2'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'), ('e2', 'e5'), ('e2', 'e6'),
                             ('e3', 'e6')]
           self.no = ['l3_4', 'l3_9']

    def Formulation(self):
        # print('Task specified by LTL formula: ' + self.formula)
        return self.workspace, self.regions, self.centers, self.obs, self.init_state, self.uni_cost, self.formula, \
               self.formula_comp, self.exclusion, self.no
