'''
This is a test script for multi-robot MILP encoding
'''

from __future__ import print_function
import networkx as nx
import sys
from lomap import Fsa, Ts, Wfse
# from lomap.algorithms import wfse_product
sys.path.append("../")
import twtl
import bound
from dfa import setDFAType, DFAType, Op, setOptimizationFlag
sys.path.append("../algorithms")

# from algorithms.new_encoding import Multi_Rob_Milp
# from milp_original import Multi_Rob_Milp
# from milp_multi_rob import Multi_Rob_Milp
# from algorithms.single_agent_working import Multi_Rob_Milp
# from time_var import Multi_Rob_Milp

from dfa_milp import Multi_Rob_Milp
import wfse_product


def fsa_constructor(task_case):

    # Define the set of atomic propositions
    '''
    T1 - T5: task regions
    O - obstacle
    B - Bridge
    '''
    ap = set(['A', 'B'])

    formula = '[H^1 A]^[0,3] * [H^1 B]^[0,6]'
    _, dfa_0, bdd = twtl.translate(formula, kind=DFAType.Normal, norm=True)
    print("DFA:", dfa_0.g.edges(data=True))
    return dfa_0


def ts_constructor():

    ts = Ts(directed=True, multi=False)
    ts.g = nx.DiGraph()
    ts.g.add_nodes_from([0,1,2,3])

    ts.g.add_weighted_edges_from([(0,1,1),(0,0,1), (0,3,1), (1,2,1), (2,2,1), (1,1,1), (2,3,1), (3,3,1)])
    ts.init = set()
    ts.init.add(0)

    ## Add lables to TS nodes

    ts.g.add_node((1), attr_dict={'prop': set(['A'])})
    ts.g.add_node((2), attr_dict={'prop': set(['B'])})

    return ts


def wfse_constructor(task_case):
    ap = set(['A', 'B']) # set of atomic propositions
    wfse = Wfse(props=ap, multi=False)
    wfse.init = set() # HACK

    # add states
    wfse.g.add_nodes_from(['q0', 'q1', 'q2', 'q3','q4','q5'])

    # add transitions
    # pass_through transitions
    pass_through_symbols = [(symbol, symbol, 1) for symbol in wfse.prop_bitmaps
                            if symbol >= 0]
    wfse.g.add_edge('q0', 'q0', attr_dict={'symbols': pass_through_symbols})

    user_preference = task_case


    if (user_preference == '2'):

        print("deletion")
        in_symbol = -1
        out_symbol= wfse.bitmap_of_props(set(['T1']))
        weighted_symbols = [(in_symbol, out_symbol, 10)]
        wfse.g.add_edge('q0', 'q1', attr_dict={'symbols': weighted_symbols})

    if (user_preference == '3'):
        print("substitution")

        # Substitute T2 by T1 with a penalty 2
        in_symbol = wfse.bitmap_of_props(set(['T2']))
        out_symbol = wfse.bitmap_of_props(set(['T1']))

        weighted_symbols = [(in_symbol, out_symbol, 5)]
        wfse.g.add_edge('q0', 'q2', attr_dict={'symbols': weighted_symbols})

        weighted_symbols = [( -1, out_symbol, 0)]
        wfse.g.add_edge('q2', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute T3 by T1 with a penalty 4
        in_symbol = wfse.bitmap_of_props(set(['T3']))
        out_symbol = wfse.bitmap_of_props(set(['T1']))
        weighted_symbols = [(in_symbol, out_symbol, 8)]
        wfse.g.add_edge('q0', 'q3', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q3', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute T4 by T1 with a penalty 6
        in_symbol = wfse.bitmap_of_props(set(['T4']))
        out_symbol = wfse.bitmap_of_props(set(['T1']))
        weighted_symbols = [(in_symbol, out_symbol, 11)]
        wfse.g.add_edge('q0', 'q4', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q4', 'q0', attr_dict={'symbols': weighted_symbols})


        # Substitute T5 by T1 with a penalty 8
        in_symbol = wfse.bitmap_of_props(set(['T5']))
        out_symbol = wfse.bitmap_of_props(set(['T1']))
        weighted_symbols = [(in_symbol, out_symbol, 14)]
        wfse.g.add_edge('q0', 'q5', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q5', 'q0', attr_dict={'symbols': weighted_symbols})

    # set the initial state
    wfse.init.add('q0')

    # set the final state
    wfse.final.add('q0')
    # wfse.final.add('q1')


    return wfse


def main():


    print("Please enter case number:\n1. Canonical\n2. Deletion\n3. Substitution")
    task_case = raw_input()

    n_agents = 1
    # n_agents = 2

    fsa = fsa_constructor(task_case)
    print("DFA finals:", fsa.final)
    ts = ts_constructor()

    wfse = wfse_constructor(task_case)
    print("wfse final:", wfse.final)
    print("===========calling============")
    product_model = wfse_product.new_wfse_times_fsa(wfse, fsa)
    print("fsa:", fsa)
    print('Product: Init:', product_model.init) # initial states
    print('Product: Final:', product_model.final) # final states
    print("product edges: ", product_model.g.edges(data=True))
    print('\n\n\n')


    Multi_Rob_Milp(n_agents, ts, product_model)

if __name__ == '__main__':
    main()