#! /usr/bin/python

# Test case study (SDC) for both task and deadline relaxation using pareto optimal Dijkstra 



from __future__ import print_function
import networkx as nx
from lomap import Fsa, Ts, Wfse, ts_times_wfse_times_fsa_pareto
from lomap.algorithms import product, dijkstra, modified_dijkstra
import matplotlib.pyplot as plt
import numpy as np
import os
import logging, sys
import StringIO
import networkx as nx
import matplotlib.pyplot as plt
import twtl
from dfa import DFAType
from synthesis import expand_duration_ts, compute_control_policy, ts_times_fsa,\
                      verify
from learning import learn_deadlines
from lomap import Ts, Wfse, ts_times_wfse_times_fsa
from platypus import NSGAII, DTLZ2
from networkx.readwrite import json_graph



def twtl_synthesis(formula, ts_file, wfse):
    _, dfa_0, dfa_inf, bdd = twtl.translate(formula, kind='both', norm=True)

    logging.debug('alphabet: {}'.format(dfa_inf.props))

    for u, v, d in dfa_inf.g.edges_iter(data=True):
        logging.debug('({}, {}): {}'.format(u, v, d))

    # dfa_inf.visualize(draw='matplotlib')
    # plt.show()

    # logging.debug('\nEnd of translate\n\n')

    # logging.info('The bound of formula "%s" is (%d, %d)!', formula, *bdd)
    # logging.info('Translated formula "%s" to normal DFA of size (%d, %d)!',
    #              formula, *dfa_0.size())
    # logging.info('Translated formula "%s" to infinity DFA of size (%d, %d)!',
    #              formula, *dfa_inf.size())

    # logging.debug('\n\nStart policy computation\n')

    ts = Ts.load(ts_file)
    # print("ts_weight", ts.g.edges(data=True))

    ets = expand_duration_ts(ts)
    # print("expand_duration_ts:", ets.g.edges(data=True))

    for name, dfa in [('normal', dfa_0), ('infinity', dfa_inf)]:
        logging.info('Constructing product automaton with %s DFA!', name)
        pa = ts_times_wfse_times_fsa_pareto(ets,wfse,dfa)  
        # logging.info('Product automaton size is: (%d, %d)', *pa.size())

        if name == 'infinity':
            for u in pa.g.nodes_iter():
                logging.debug('{} -> {}'.format(u, pa.g.neighbors(u)))

        product_model = pa
        init_states = list(product_model.init)
        # print("init_states:", init_states)
        final_states = list(product_model.final)

        o1 = [] # List of task relaxation costs corresponding to different values of the parameter
        o2 = [] # List of deadline relaxation costs corresponding to different values of the parameter
        mul_obj_cost = [] # List of linear combinations of task and deadline relaxations (pareto costs)
        obj1_prev = 0 # Task relaxation cost for the previous value of parameter
        obj2_prev = 0 # Deadline relaxation cost for the previous value of parameter


        # Iterate over all final states and find the correponding path lenths and paths

        for parameter in np.arange(0.1,1,0.1): 
            print("-----------------------------------")

            dijkstra_length = []    # This list stores the modified Dijkstra path lengths for all final states 
            old_dijkstra_length = []  #  This list stores the Dijkstra path lengths for all final states 
 

            for each_state in product_model.final:

                # print("final_state",each_state) 
                # print("param:", parameter)  

                # The old cost (using the original Dijkstra implementation) is taken into account just for comparison      
                old_length = nx.dijkstra_path_length(product_model.g, init_states[0], each_state,weight='weight')
                length= modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0], each_state,weight='weight', parameter=parameter, obj_data=True)

                dijkstra_length.append(length)
                old_dijkstra_length.append(old_length)

            # print("length:",dijkstra_length)
            # print("old_len", old_dijkstra_length)


            if (not dijkstra_length):
                robot_current_state = ts.init
                print("No feasible final states, deleting the tasks...")
                return

            # Get the index corresponding to the minimum cost and retrieve the corresponding final state

            pa_optimal_index = np.argmin(dijkstra_length)
            pa_optimal_final_state = final_states[pa_optimal_index]
            print("pa_optimal_final_state:", pa_optimal_final_state)

            old_optimal_index = np.argmin(old_dijkstra_length)
            old_optimal_final_state = final_states[old_optimal_index]
            print("old_optimal_final_state:", old_optimal_final_state)


            # Find out the min length path with the optimal final state as a target using Dijkstra 
            old_optimal_path = nx.dijkstra_path(product_model.g, init_states[0],old_optimal_final_state,weight='weight')
            pa_optimal_path, obj_one, obj_two = modified_dijkstra.dijkstra_path(product_model.g, init_states[0],target= pa_optimal_final_state,weight='weight',parameter=parameter, obj_data=True)

            # pa_optimal_cost = nx.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
            pa_optimal_cost = modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight',parameter=parameter)
            mul_obj_cost.append(pa_optimal_cost)

            o1.append(obj_one)
            o2.append(obj_two)

            # print("TOTAL COST for parameter: ", pa_optimal_cost, parameter)
            # pa_optimal_path = nx.bidirectional_dijkstra(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
            # pa_optimal_path = nx.astar_path(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')

            # print("Optimal_path", pa_optimal_path)
            # print("old_opt_path", old_optimal_path)

            # Obtain the individual optimal paths for each component 
            ts_optimal_path, wfse_state_path, fsa_state_path = zip(*pa_optimal_path)
            old_ts_optimal_path, old_wfse_state_path, old_fsa_state_path = zip(*old_optimal_path)

            # print("opt_ts:",ts_optimal_path)
            # ts_tree = nx.DiGraph([ts_optimal_path])
            # data = json_graph.tree_data(ts_tree,'Base')
            # data_tree = json_graph.tree_graph(data)

            # To obtain each solution only once
            if (obj_one not in o1) & (obj_two not in o2): 

                # print('TS: Optimal Trajectory:', ts_optimal_path)
                print('WFSE: Optimal Trajectory:', wfse_state_path)
                # print('FSA: Optimal Trajectory:', fsa_state_path)


                # print('old TS: Optimal Trajectory:', old_ts_optimal_path)
                print('old WFSE: Optimal Trajectory:', old_wfse_state_path)
                # print('old FSA: Optimal Trajectory:', old_fsa_state_path)

            # print('WFSE_nodes_size:', wfse.g.number_of_nodes())
            # print('wfse_edges_size:', wfse.g.number_of_edges())
            # print('PA_nodes_size:', product_model.g.number_of_nodes())
            # print('PA_edges_size:', product_model.g.number_of_edges())


            print('Symbol translations:')
            for ts_state, state, next_state in zip(ts_optimal_path[1:], pa_optimal_path,
                                                   pa_optimal_path[1:]):
                transition_data = product_model.g[state][next_state]
                original_symbol, transformed_symbol = transition_data['prop']
                # if original_symbol ==  set([]) & transformed_symbol ==  set([]): 
                #     continue
                print(ts_state, ':', original_symbol, '->', transformed_symbol)

            obj1_prev = obj_one
            obj2_prev = obj_two



    print("obj_one:", o1)
    print("obj_two:", o2)
    # print("pareto cost:", mul_obj_cost)


    x = np.arange(0.1,1,0.1)
    y = mul_obj_cost 

    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    ax1.plot(x, y,'bo')
    ax1.set_xlabel('Parameter values')
    ax1.set_ylabel('Total_cost')

    ax2 = fig.add_subplot(212)
    ax2.plot(o1, o2, 'ro')
    ax2.set_xlabel('Task relaxation')
    ax2.set_ylabel('Deadline relaxation')

    plt.show()



def wfse_constructor(task_case):
    ap = set(['A', 'B', 'C', 'D','E','O','B']) # set of atomic propositions
    wfse = Wfse(props=ap, multi=False)
    wfse.init = set() # HACK

    # add states
    wfse.g.add_nodes_from(['q0', 'q1', 'q2', 'q3','q4','q5','q6','q7','q8','q9','q10','q11','q12'])

    # add transitions
    pass_through_symbols = [(symbol, symbol, 1) for symbol in wfse.prop_bitmaps
                            if symbol >= 0]
    # print('pass through symbols:', pass_through_symbols)
    wfse.g.add_edge('q0', 'q0', attr_dict={'symbols': pass_through_symbols})
    wfse.g.add_edge('q4', 'q4', attr_dict={'symbols': pass_through_symbols})


    user_preference = task_case


    if (user_preference == '2'): 

        print("deletion")

        # in_symbol = wfse.bitmap_of_props(set(['E']))
        in_symbol = -1
        # out_symbol = wfse.bitmap_of_props(set())
        out_symbol= wfse.bitmap_of_props(set(['A'])) 
        # weighted_symbols = [(in_symbol, out_symbol, 2)]
        weighted_symbols = [(in_symbol, out_symbol, 10)]
        wfse.g.add_edge('q0', 'q1', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, -1, 0)]    
        wfse.g.add_edge('q1', 'q0', attr_dict={'symbols': weighted_symbols})

    if (user_preference == '3'): 
        print("substitution")


        ## E & C & B [q5,q6,q7]

        in_symbol = wfse.bitmap_of_props(set(['E']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 1)]
        wfse.g.add_edge('q0', 'q5', attr_dict={'symbols': weighted_symbols})


        weighted_symbols = [(in_symbol, -1, 0)]
        wfse.g.add_edge('q5', 'q0', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set())
        out_symbol = wfse.bitmap_of_props(set())
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q5', 'q5', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set(['C']))
        # out_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = -1
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q5', 'q6', attr_dict={'symbols': weighted_symbols})


        weighted_symbols = [(-1, -1, 0)]
        wfse.g.add_edge('q6', 'q0', attr_dict={'symbols': weighted_symbols})


        in_symbol = wfse.bitmap_of_props(set())
        out_symbol = wfse.bitmap_of_props(set())
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q6', 'q6', attr_dict={'symbols': weighted_symbols})


        in_symbol = wfse.bitmap_of_props(set(['B']))
        # out_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = -1
        weighted_symbols = [(in_symbol,out_symbol,0)]
        wfse.g.add_edge('q6', 'q7', attr_dict={'symbols': weighted_symbols})



        # C & B [q3,q4]

        in_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 2)]
        wfse.g.add_edge('q0', 'q3', attr_dict={'symbols': weighted_symbols})


        weighted_symbols = [(in_symbol, -1, 0)]
        wfse.g.add_edge('q3', 'q0', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set())
        out_symbol = wfse.bitmap_of_props(set())
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q3', 'q3', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set(['B']))
        # out_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = -1
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q3', 'q4', attr_dict={'symbols': weighted_symbols})


  
        # E & B   [q1,q2] 

        in_symbol = wfse.bitmap_of_props(set(['E']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 3)]
        wfse.g.add_edge('q0', 'q1', attr_dict={'symbols': weighted_symbols})


        weighted_symbols = [(in_symbol, -1, 0)]
        wfse.g.add_edge('q1', 'q0', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set())
        out_symbol = wfse.bitmap_of_props(set())
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q1', 'q1', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set(['B']))
        # out_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = -1
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q1', 'q2', attr_dict={'symbols': weighted_symbols})


        # E & C [q8,q9]


        in_symbol = wfse.bitmap_of_props(set(['E']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 4.5)]
        wfse.g.add_edge('q0', 'q8', attr_dict={'symbols': weighted_symbols})


        weighted_symbols = [(in_symbol, -1, 0)]
        wfse.g.add_edge('q8', 'q0', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set())
        out_symbol = wfse.bitmap_of_props(set())
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q8', 'q8', attr_dict={'symbols': weighted_symbols})

        in_symbol = wfse.bitmap_of_props(set(['C']))
        # out_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = -1
        weighted_symbols = [(in_symbol,out_symbol,1)]
        wfse.g.add_edge('q8', 'q9', attr_dict={'symbols': weighted_symbols})


        # B 
        in_symbol = wfse.bitmap_of_props(set(['B']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 12)]
        wfse.g.add_edge('q0', 'q10', attr_dict={'symbols': weighted_symbols})



        # C 
        in_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 14)]
        wfse.g.add_edge('q0', 'q11', attr_dict={'symbols': weighted_symbols})



        # E 
        in_symbol = wfse.bitmap_of_props(set(['E']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        # out_symbol = -1
        weighted_symbols = [(in_symbol, out_symbol, 18)]
        wfse.g.add_edge('q0', 'q12', attr_dict={'symbols': weighted_symbols})





        # weighted_symbols = [(-1, -1, 1)]
        # wfse.g.add_edge('q2', 'q0', attr_dict={'symbols': weighted_symbols})

    # set the initial state
    wfse.init.add('q0') 

    # set the final state
    wfse.final.add('q7')
    wfse.final.add('q2')

    wfse.final.add('q9')
    wfse.final.add('q4')
    wfse.final.add('q10')
    wfse.final.add('q11')
    wfse.final.add('q12')



    # nx.draw(wfse.g, with_labels=True)
    # nx.draw_networkx_edge_labels(wfse.g,pos=nx.spring_layout(wfse.g))
    # plt.show()

    return wfse


def main():


    print("Please enter case number:\n1. Canonical\n2. Deletion\n3. Substitution")
    task_case = raw_input()


    # phi_sdc = '[H^13 !O] ^ [0,15] & [H^2 A] ^ [0,8]' 

    # case1_synthesis(phi_sdc, '../data/ts_synthesis.yaml')


    # ts = ts_constructor_TR()
    # print(ts)
    wfse = wfse_constructor(task_case)
    # print(wfse)


    phi_sdc = '(H^15 !O) & [H^1 A] ^ [0,7]'
    # phi_sdc = '([!O] ^ [0,15] & [A] ^ [0,10])'
    # phi_sdc =  '[A] ^ [0,15]'

    # phi_sdc = '([H^1 E]^ [0,5] & [H^1 C] ^[0,10])' # Path exists

    # phi_sdc = '[H^2 D] ^ [0,5]'

    # phi_sdc = '[H^1 A] ^ [0,13]' 


    
    twtl_synthesis(phi_sdc, '../data/sdc_word.yaml', wfse)



if __name__ == '__main__':
    main()
