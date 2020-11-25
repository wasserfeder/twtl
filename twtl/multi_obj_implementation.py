#! /usr/bin/python

# Test case for using Weighted Finite State Error Systems for task substitution
# Copyright (C) 2020, Cristian-Ioan Vasile (cvasile@lehigh.edu)
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.



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



def ts_constructor_TR():

    #Create a transition system for task relaxation part (weights are not unit)

    ts = Ts(directed=True, multi=False)
    ts.g = nx.DiGraph()
    ts.g.add_nodes_from([0,1,2,3,4,5,6,7,8,9,10,11,12,13])

    ts.g.add_weighted_edges_from([(0,6,1), (6,4,1),(6,7,1),(6,8,3),(7,5,1),(8,9,1),(7,10,3),
                                  (9,10,1),(10,11,2),(11,1,1),(8,10,4),(8,12,1),
                                  (12,3,1),(12,13,3),(13,2,1)])

    ts.init[(0)] = 1

    ## Add lables to TS nodes

    ts.g.add_node((1), attr_dict={'prop': set(['A'])})  
    ts.g.add_node((2), attr_dict={'prop': set(['B'])})
    ts.g.add_node((3), attr_dict={'prop': set(['C'])})
    ts.g.add_node((4), attr_dict={'prop': set(['D'])})
    ts.g.add_node((5), attr_dict={'prop': set(['E'])})
    ts.g.add_node((10), attr_dict={'prop': set(['O'])})
    ts.g.add_node((9), attr_dict={'prop': set(['Br'])})


    ## Visualize the TS
    # nx.draw(ts.g , with_labels=True, node_color='b')
    # plt.show()

    return ts

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
    print("ts_weight", ts.g.edges(data=True))

    ets = expand_duration_ts(ts)
    print("expand_duration_ts:", ets.g.edges(data=True))

    for name, dfa in [('normal', dfa_0), ('infinity', dfa_inf)]:
        logging.info('Constructing product automaton with %s DFA!', name)

        pa = ts_times_wfse_times_fsa_pareto(ts,wfse,dfa)
        # print("")
        # print("product_ts:",pa.g.edges(data=True))


        pa = ts_times_wfse_times_fsa(ets, wfse, dfa)
        # print("")
        # print("product_ets:",pa.g.edges(data=True))

        # logging.info('Product automaton size is: (%d, %d)', *pa.size())

        if name == 'infinity':
            for u in pa.g.nodes_iter():
                logging.debug('{} -> {}'.format(u, pa.g.neighbors(u)))

            # pa.visualize(draw='matplotlib')
            # plt.show()

        # # compute optimal path in PA and then project onto the TS
        # policy, output, tau = compute_control_policy(pa, dfa, dfa.kind)
        # logging.info('Max deadline: %s', tau)
        # if policy is not None:
        #     logging.info('Generated output word is: %s', [tuple(o) for o in output])

        #     policy = [x for x in policy if x not in ets.state_map]
        #     out = StringIO.StringIO()
        #     # for u, v in zip(policy[:-1], policy[1:]):
        #     #     print>>out, u, '->',s ts.g[u][v]['duration'], '->',v
        #     print>>out, policy[-1],
        #     logging.info('Generated control policy is: %s', out.getvalue())
        #     out.close()

        #     logging.info('Relaxation is: %s',
        #                  twtl.temporal_relaxation(output, formula=formula))
        # else:
        #     logging.info('No control policy found!')




        # get initial state in product model -- should be only one
        # Convert the sets of initial and final states into lists
        # parameter = 0.5
        product_model = pa
        init_states = list(product_model.init)
        final_states = list(product_model.final)

        o1 = []
        o2 = []
        mul_obj_cost = []


        # Iterate over all final states and find the correponding path lenths and paths

        for parameter in np.arange(0,1.1,0.1): 
            print("-----------------------------------")

            dijkstra_length = []    # This list stores the Dijkstra path lengths for all final states 
            old_dijkstra_length = []
 

            for each_state in product_model.final:

                print("final_state",each_state) 
                print("param:", parameter)       
                old_length = nx.dijkstra_path_length(product_model.g, init_states[0], each_state,weight='weight')
                length= modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0], each_state,weight='weight', parameter=parameter, obj_data=True)
                # length, o1, o2= modified_dijkstra._dijkstra(product_model.g, init_states[0], each_state,weight='weight', parameter=parameter)

                # print(init_states[0][0])

                dijkstra_length.append(length)
                old_dijkstra_length.append(old_length)

            print("length:",dijkstra_length)
            print("old_len", old_dijkstra_length)


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


            # pa_optimal_path = modified_dijkstra.dijkstra_path(product_model.g, init_states[0],target= pa_optimal_final_state,weight='weight',parameter=parameter, obj_data=True)



            # pa_optimal_path = dijkstra.source_to_target_dijkstra(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')

            # pa_optimal_cost = nx.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
            pa_optimal_cost = modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight',parameter=parameter)
            mul_obj_cost.append(pa_optimal_cost)

            # obj_one = nx.dijkstra_path_length(ets.g, init_states[0][0], pa_optimal_final_state[0][0],weight='weight')
            # obj_two = nx.dijkstra_path_length(product_model.g, init_states[0], pa_optimal_final_state ,weight='weight')
            o1.append(obj_one)
            o2.append(obj_two)

            print("TOTAL COST for parameter: ", pa_optimal_cost, parameter)
            # pa_optimal_path = nx.bidirectional_dijkstra(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
            # pa_optimal_path = nx.astar_path(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')

            print("Optimal_path", pa_optimal_path)
            print("old_opt_path", old_optimal_path)

            # Obtain the individual optimal paths for each component 
            ts_optimal_path, wfse_state_path, fsa_state_path = zip(*pa_optimal_path)

            print('TS: Optimal Trajectory:', ts_optimal_path)
            print('WFSE: Optimal Trajectory:', wfse_state_path)
            print('FSA: Optimal Trajectory:', fsa_state_path)

            old_ts_optimal_path, old_wfse_state_path, old_fsa_state_path = zip(*old_optimal_path)

            print('old TS: Optimal Trajectory:', old_ts_optimal_path)
            print('old WFSE: Optimal Trajectory:', old_wfse_state_path)
            print('old FSA: Optimal Trajectory:', old_fsa_state_path)

            # print('WFSE_nodes_size:', wfse.g.number_of_nodes())
            # print('wfse_edges_size:', wfse.g.number_of_edges())
            # print('PA_nodes_size:', product_model.g.number_of_nodes())
            # print('PA_edges_size:', product_model.g.number_of_edges())


            print('Symbol translations:')
            for ts_state, state, next_state in zip(ts_optimal_path[1:], pa_optimal_path,
                                                   pa_optimal_path[1:]):
                transition_data = product_model.g[state][next_state]
                original_symbol, transformed_symbol = transition_data['prop']
                print(ts_state, ':', original_symbol, '->', transformed_symbol)


    print("obj_one:", o1)
    print("obj_two:", o2)
    print("pareto cost:", mul_obj_cost)

    x = np.arange(0,1.1,0.1)
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
    wfse.g.add_nodes_from(['q0', 'q1', 'q2', 'q3','q4'])

    # add transitions
    pass_through_symbols = [(symbol, symbol, 1) for symbol in wfse.prop_bitmaps
                            if symbol >= 0]
    # print('pass through symbols:', pass_through_symbols)
    wfse.g.add_edge('q0', 'q0', attr_dict={'symbols': pass_through_symbols})

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

        # Substitute B by A with a penalty 2
        in_symbol = wfse.bitmap_of_props(set(['B']))
        out_symbol = wfse.bitmap_of_props(set(['A']))

        weighted_symbols = [(in_symbol, out_symbol, 5)]
        wfse.g.add_edge('q0', 'q1', attr_dict={'symbols': weighted_symbols})

        weighted_symbols = [( -1, out_symbol, 0)] 
        wfse.g.add_edge('q1', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute C by A with a penalty 4
        in_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        weighted_symbols = [(in_symbol, out_symbol, 10)]
        wfse.g.add_edge('q0', 'q2', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]    
        wfse.g.add_edge('q2', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute D by A with a penalty 6
        in_symbol = wfse.bitmap_of_props(set(['D']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        weighted_symbols = [(in_symbol, out_symbol, 20)]
        wfse.g.add_edge('q0', 'q3', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q3', 'q0', attr_dict={'symbols': weighted_symbols})


        # Substitute E by A with a penalty 8
        in_symbol = wfse.bitmap_of_props(set(['E']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        weighted_symbols = [(in_symbol, out_symbol, 30)]
        wfse.g.add_edge('q0', 'q4', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q4', 'q0', attr_dict={'symbols': weighted_symbols})

    # set the initial state
    wfse.init.add('q0')

    # set the final state
    wfse.final.add('q0')

    # nx.draw(wfse.g, with_labels=True)
    # nx.draw_networkx_edge_labels(wfse.g,pos=nx.spring_layout(wfse.g))
    # plt.show()

    return wfse


def main():


    print("Please enter case number:\n1. Canonical\n2. Deletion\n3. Substitution")
    task_case = raw_input()


    phi_sdc = '[H^13 !O] ^ [0,15] & [H^2 A] ^ [0,8]' 

    # case1_synthesis(phi_sdc, '../data/ts_synthesis.yaml')


    ts = ts_constructor_TR()
    # print(ts)
    wfse = wfse_constructor(task_case)
    # print(wfse)
    twtl_synthesis(phi_sdc, '../data/sdc_case_study.yaml', wfse)

    phi_sdc = '[H^5 C] ^ [0,25]'

    twtl_synthesis(phi_sdc, '../data/sdc_case_study.yaml', wfse)

    phi_sdc = '[H^5 C] ^ [0,15] & [H^2 B] ^ [0,30]'

    twtl_synthesis(phi_sdc, '../data/sdc_case_study.yaml', wfse)
    # product_model = ts_times_wfse_times_fsa(ts, wfse, dfa)

    # print('Product: Init:', product_model.init) # initial states
    # print('Product: Final:', product_model.final) # final states
    # print('product_model_edges:', product_model.g.edges(data=True))
    # print('TS_edge_data:', ts.g.edges(data=True))
    # print('\n\n\n')


    # # get initial state in product model -- should be only one
    # # Convert the sets of initial and final states into lists
    # init_states = list(product_model.init)
    # final_states = list(product_model.final)
    # dijkstra_length = []    # This list stores the Dijkstra path lengths for all final states 


    # for parameter in np.arange(0.1,1,0.1):

    #     # Iterate over all final states and find the correponding path lenths and paths
    #     for each_state in product_model.final:
    #         print(each_state)        
    #         # length = nx.dijkstra_path_length(product_model.g, init_states[0], each_state,weight='weight')
    #         length = modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0], each_state,weight='weight', parameter=parameter)


    #         dijkstra_length.append(length)
    #     print("length:",dijkstra_length)


    #     if (not dijkstra_length):
    #         robot_current_state = ts.init
    #         print("No feasible final states, deleting the tasks...")
    #         return

    #     # Get the index corresponding to the minimum cost and retrieve the corresponding final state

    #     pa_optimal_index = np.argmin(dijkstra_length)
    #     pa_optimal_final_state = final_states[pa_optimal_index]
    #     print("pa_optimal_final_state:", pa_optimal_final_state)

    #     # Find out the min length path with the optimal final state as a target using Dijkstra 


    #     # pa_optimal_path = nx.dijkstra_path(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
    #     pa_optimal_path = modified_dijkstra.dijkstra_path(product_model.g, init_states[0],pa_optimal_final_state,weight='weight',parameter=parameter)


    #     # pa_optimal_path = dijkstra.source_to_target_dijkstra(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')

    #     # pa_optimal_cost = nx.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
    #     pa_optimal_cost = modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight',parameter=parameter)

    #     print("TOTAL COST:", pa_optimal_cost)
    #     # pa_optimal_path = nx.bidirectional_dijkstra(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
    #     # pa_optimal_path = nx.astar_path(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')

    #     print("Optimal_path", pa_optimal_path)

    #     # Obtain the individual optimal paths for each component 
    #     ts_optimal_path, wfse_state_path, fsa_state_path = zip(*pa_optimal_path)

    #     print('TS: Optimal Trajectory:', ts_optimal_path)
    #     print('WFSE: Optimal Trajectory:', wfse_state_path)
    #     print('FSA: Optimal Trajectory:', fsa_state_path)

    #     print('WFSE_nodes_size:', wfse.g.number_of_nodes())
    #     print('wfse_edges_size:', wfse.g.number_of_edges())
    #     print('PA_nodes_size:', product_model.g.number_of_nodes())
    #     print('PA_edges_size:', product_model.g.number_of_edges())


    #     print('Symbol translations:')
    #     for ts_state, state, next_state in zip(ts_optimal_path[1:], pa_optimal_path,
    #                                            pa_optimal_path[1:]):
    #         transition_data = product_model.g[state][next_state]
    #         original_symbol, transformed_symbol = transition_data['prop']
    #         print(ts_state, ':', original_symbol, '->', transformed_symbol)


if __name__ == '__main__':
    main()
