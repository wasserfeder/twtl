#! /usr/bin/python

# Test case study (SDC) for both task and deadline relaxation using pareto optimal Dijkstra


from __future__ import print_function
from __future__ import division
import networkx as nx
from lomap import Fsa, Ts, Wfse, ts_times_wfse_times_fsa_pareto
from lomap.algorithms import product, dijkstra, modified_dijkstra, parametric_shortest_path
import matplotlib.pyplot as plt
import numpy as np
import os
import logging, sys
import StringIO
import networkx as nx
import matplotlib.pyplot as plt
import twtl
from dfa import DFAType
from synthesis import expand_duration_ts, compute_control_policy, ts_times_fsa, \
    verify
from learning import learn_deadlines
from lomap import Ts, Wfse, ts_times_wfse_times_fsa
from platypus import NSGAII, DTLZ2
from networkx.readwrite import json_graph
import intervals as I

split_list = []
d1_final = {}
d2_final = {}
biobj = {}
# def ts_constructor_TR():

#     #Create a transition system for task relaxation part (weights are not unit)

#     ts = Ts(directed=True, multi=False)
#     ts.g = nx.DiGraph()
#     ts.g.add_nodes_from([0,1,2,3,4,5,6,7,8,9,10,11,12,13])

#     ts.g.add_weighted_edges_from([(0,6,1), (6,4,1),(6,7,1),(6,8,3),(7,5,1),(8,9,1),(7,10,3),
#                                   (9,10,1),(10,11,2),(11,1,1),(8,10,4),(8,12,1),
#                                   (12,3,1),(12,13,3),(13,2,1)])

#     ts.init[(0)] = 1

#     ## Add lables to TS nodes

#     ts.g.add_node((1), attr_dict={'prop': set(['A'])})
#     ts.g.add_node((2), attr_dict={'prop': set(['B'])})
#     ts.g.add_node((3), attr_dict={'prop': set(['C'])})
#     ts.g.add_node((4), attr_dict={'prop': set(['D'])})
#     ts.g.add_node((5), attr_dict={'prop': set(['E'])})
#     ts.g.add_node((10), attr_dict={'prop': set(['O'])})
#     ts.g.add_node((9), attr_dict={'prop': set(['Br'])})


#     ## Visualize the TS
#     # nx.draw(ts.g , with_labels=True, node_color='b')
#     # plt.show()

#     return ts

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
    optimal_intervals = []

    for name, dfa in [('normal', dfa_0), ('infinity', dfa_inf)]:
        logging.info('Constructing product automaton with %s DFA!', name)
        pa = ts_times_wfse_times_fsa_pareto(ets, wfse, dfa)
        # logging.info('Product automaton size is: (%d, %d)', *pa.size())
        d1_old = 10000000
        d2_old = 10000000
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

        product_model = pa
        init_states = list(product_model.init)
        final_states = list(product_model.final)
        print("final_states:",final_states)

        o1 = []  # List of task relaxation costs corresponding to different values of the parameter
        o2 = []  # List of deadline relaxation costs corresponding to different values of the parameter
        mul_obj_cost = []  # List of linear combinations of task and deadline relaxations (pareto costs)
        new_cost = []
        param_list = []
        obj1_prev = 0  # Task relaxation cost for the previous value of parameter
        obj2_prev = 0  # Deadline relaxation cost for the previous value of parameter
        parameter_search_list = []  # List of parameter values on which to perform binary search

        # print("pa_states:", pa.g.nodes())

        d1, d2, p = parametric_shortest_path.dijkstra2(product_model.g, init_states[0])
        # print("distance1:", d1[('N', 'q0', 12)])
        # print("distance2:", d2[('N', 'q0', 12)])
        # print("pre: ", p[('N', 'q0', 12)])

        for predecessor in d1[('P', 'q0', 12)]:
            o1.append(d1[('P', 'q0', 12)][predecessor])
            o2.append(d2[('N', 'q0', 12)][predecessor])
            print("here!", predecessor)

            for param in np.arange(predecessor.lower, predecessor.upper, 0.01):
                print("param:", param)
                print("obj1:", d1[('N', 'q0', 12)][predecessor])
                print("obj2:", d2[('N', 'q0', 12)][predecessor])

                if param == 0 or param == 1:
                    print("This is working!")
                    continue
                cost = param * d1[('N', 'q0', 12)][predecessor] + (1-param) * d2[('N', 'q0', 12)][predecessor]
                print("cost:", cost)
                new_cost.append(cost)
                param_list.append(param)

        print("cost_list:", new_cost)

        # for each_state in product_model.final:
        #     biobj[each_state] = {}
        #     d1_final[each_state] = {}
        #     d2_final[each_state] = {}
        #
        #     print("final_state",each_state)
        #     # print("param:", parameter)
        #     print("obj1_dij2:", d1[each_state])
        #     print("obj2_dij2:", d2[each_state])
        #     d1_final[each_state][Iv] =  d1[each_state][Iv]
        #     d2_final[each_state][Iv] =  d2[each_state][Iv]



            # Iv_new, d1_new, d2_new = relax(each_state,Iu, Iv,Iv_prev, d1_old, d2_old, d1[each_state][Iv], d2[each_state][Iv])
            # print("new Iv:", Iv_new)
            # # d1[each_state][Iv_new] =
            # # Iv = Iv_new
            # # biobj[each_state][Iv_new] = 1
            # d1_old = d1_new
            # d2_old = d2_new
            # # Iv_prev = Iv_new
            #
            # print("-------------------------------------------")



        # print("final d1: ", d1_final)
        # print("final d2: ", d2_final)

            # Iterate over all final states and find the correponding path lenths and paths

        for parameter in np.arange(0.1, 1, 0.1):
            print("-----------------------------------")

            dijkstra_length = []  # This list stores the modified Dijkstra path lengths for all final states
            old_dijkstra_length = []  # This list stores the Dijkstra path lengths for all final states
            Iu = I.closed(0,1)
            Iv = I.closed(0,1)
        # 
            for each_state in product_model.final:
                print("final_state",each_state)
                # print("param:", parameter)
                # print("obj1_dij2:", d1[each_state])
                # print("obj2_dij2:", d2[each_state])
                # relax(Iu, Iv, d1_old, d2_old, d1[each_state][Iv], d1[each_state][Iv] )
                # d1_old = d1[each_state][Iv]
                # d2_old = d2[each_state][Iv]


                # The old cost (using the original Dijkstra implementation) is taken into account just for comparison
                old_length = nx.dijkstra_path_length(product_model.g, init_states[0], each_state, weight='weight')
                length = modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0], each_state,
                                                                weight='weight', parameter=parameter, obj_data=True)

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
            # print("pa_optimal_final_state:", pa_optimal_final_state)
            # print("optimal_interval:", d1_final[pa_optimal_final_state])
            # optimal_intervals.append(d1_final[pa_optimal_final_state])
            # optimal_intervals[pa_optimal_final_state] = d1_final[pa_optimal_final_state]

            old_optimal_index = np.argmin(old_dijkstra_length)
            old_optimal_final_state = final_states[old_optimal_index]
            # print("old_optimal_final_state:", old_optimal_final_state)

            # Find out the min length path with the optimal final state as a target using Dijkstra
            old_optimal_path = nx.dijkstra_path(product_model.g, init_states[0], old_optimal_final_state,
                                                weight='weight')
            pa_optimal_path, obj_one, obj_two = modified_dijkstra.dijkstra_path(product_model.g, init_states[0],
                                                                                target=pa_optimal_final_state,
                                                                                weight='weight', parameter=parameter,
                                                                                obj_data=True)

            # pa_optimal_cost = nx.dijkstra_path_length(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
            pa_optimal_cost = modified_dijkstra.dijkstra_path_length(product_model.g, init_states[0],
                                                                     pa_optimal_final_state, weight='weight',
                                                                     parameter=parameter)
            # mul_obj_cost.append(pa_optimal_cost)

            # Obtain the individual optimal paths for each component
            ts_optimal_path, wfse_state_path, fsa_state_path = zip(*pa_optimal_path)
            old_ts_optimal_path, old_wfse_state_path, old_fsa_state_path = zip(*old_optimal_path)

            # To obtain each solution only once
            # if (obj_one not in o1) | (obj_two not in o2):
            #     print("\n")
            #     parameter_search_list.append(parameter)
                # print("Solution corresponding to lambda = ", parameter)

                # print('TS: Optimal Trajectory:', ts_optimal_path)
                # print('WFSE: Optimal Trajectory:', wfse_state_path)
                # print('FSA: Optimal Trajectory:', fsa_state_path)

                # print('old TS: Optimal Trajectory:', old_ts_optimal_path)
                # print('old WFSE: Optimal Trajectory:', old_wfse_state_path)
                # print('old FSA: Optimal Trajectory:', old_fsa_state_path)

                # print("\n")

            # o1.append(obj_one)
            # o2.append(obj_two)

            # print("TOTAL COST for parameter: ", pa_optimal_cost, parameter)
            # pa_optimal_path = nx.bidirectional_dijkstra(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')
            # pa_optimal_path = nx.astar_path(product_model.g, init_states[0],pa_optimal_final_state,weight='weight')

            # print("Optimal_path", pa_optimal_path)
            # print("old_opt_path", old_optimal_path)

            # print("opt_ts:", ts_optimal_path)
            # ts_tree = nx.DiGraph([ts_optimal_path])
            # data = json_graph.tree_data(ts_tree,'Base')
            # data_tree = json_graph.tree_graph(data)

            # print('WFSE_nodes_size:', wfse.g.number_of_nodes())
            # print('wfse_edges_size:', wfse.g.number_of_edges())
            # print('PA_nodes_size:', product_model.g.number_of_nodes())
            # print('PA_edges_size:', product_model.g.number_of_edges())

            # print('Symbol translations:')
            # for ts_state, state, next_state in zip(ts_optimal_path[1:], pa_optimal_path,
            #                                        pa_optimal_path[1:]):
            #     transition_data = product_model.g[state][next_state]
            #     original_symbol, transformed_symbol = transition_data['prop']
            #     print(ts_state, ':', original_symbol, '->', transformed_symbol)

            # obj1_prev = obj_one
            # obj2_prev = obj_two

    # print("obj_one:", o1)
    # print("obj_two:", o2)
    # print("pareto cost:", mul_obj_cost)

    # print("list of diverging lambda values: ", parameter_search_list)


    x = param_list
    y = new_cost


    fig1,ax = plt.subplots()
    ax.plot(x, y, 'bo')
    ax.set_xlabel('Parameter values', fontsize=16)
    ax.set_ylabel('Total_cost', fontsize=16)
    ax.grid(b=True)


    
    fig2, ax2 = plt.subplots()
    ax2.plot(o1, o2, 'ro')
    ax2.set_xlabel('Task relaxation', fontsize=16)
    ax2.set_ylabel('Deadline relaxation', fontsize=16)
    ax2.grid()
    plt.grid(b=True)
    plt.show()


def find_split(d1_temp, d2_temp, d1_value, d2_value):
    print("values_received: ", d1_temp, d2_temp, d1_value, d2_value)
    denominator = d1_value - d1_temp + d2_temp - d2_value
    print("denom:", denominator)
    assert denominator != 0
    num = d2_temp - d2_value
    print("num:", num)
    split = (d2_temp - d2_value) / denominator
    print("split_og:", split)

    return split, denominator


def relax(v, Iu, Iv, Iv_prev, d1, d2, d1_temp, d2_temp):
    '''This function compares the outputs the intervals in which a particular path is optimal and the corresponding cost'''

    global Iv_new
    i_int = Iu& Iv
    print("i_int:", i_int)
    assert not i_int.is_empty()
    #
    # weights = G.get_edge_data(u, v)
    # # d1_temp = d1[u][Iu] + weights['weight1']
    # # d2_temp = d2[u][Iu] + weights['weight2']
    # d1_temp = d1[u][Iu] + weights['weight']
    # d2_temp = d2[u][Iu] + 1

    # print("new d: ", d1_temp, d2_temp)
    param_low, param_high = i_int.lower, i_int.upper
    J_l = param_low * d1 + (1 - param_low) * d2
    J_h = param_high * d1 + (1 - param_high) \
          * d2

    J_l_new = param_low * d1_temp + (1 - param_low) * d2_temp
    J_h_new = param_high * d1_temp + (1 - param_high) * d2_temp

    print("costs: ", J_l, J_h, J_l_new, J_h_new)

    if (J_l <= J_l_new) and (J_h <= J_h_new):  # no improvement
        print("no improvement")
        pass

    ## Todo: include conditions for combinations of equal costs

    elif (J_l > J_l_new) and (J_h > J_h_new):  # all improve
        Iv1 = i_int
        Iv2 = Iv - Iv1
        # print(Iv2)
        print("improvement")
        # print("p:",p)
        # print(I.empty())

        if Iv2.is_empty():
            print("Iv2 is phi and improvement")

            Iv_new = i_int

            d1_final[v][Iv] = d1_temp
            d2_final[v][Iv] = d2_temp
            # biobj[v][Iv] =
            # p[v][Iv] = u

        else:  ## double check this part
            print("improvement")
            Iv_new = Iv2
            d1_final[v][Iv2] = d1
            d2_final[v][Iv2] = d2
            del d1_final[v][Iv]
            del d2_final[v][Iv]
            # d1[v][Iv2] = d1_temp
            # d2[v][Iv2] = d2_temp
            #
            # p[v][Iv2] = p[v][Iv]
            # del p[v][Iv]
            # p[u][Iv] = u
    else:

        print("found a cut")
        # print("old_values:", d1[v][Iv], d2[v][Iv])
        split, denom = find_split(d1_temp, d2_temp, d1, d2)
        print("split:", split)
        print("denom:", denom)
        split_list.append(split)
        Iv1 = i_int
        Iv2 = Iv - i_int
        print("Iv2: ", Iv2)

        if Iv2.is_empty():
            print("Iv2 is phi")
            if denom > 0:
                Iv_new = I.closed(split, param_high)
                print("new_iv:", Iv_new)
                d1_final[v][Iv_new] = d1_temp
                d2_final[v][Iv_new] = d2_temp
                # p[v][Iv_new] = u
                #
                # Iv_remaining = I.closed(Iv.lower, split)
                # d1_final[v][Iv_remaining] = d1
                # d2_final[v][Iv_remaining] = d2
                del d1_final[v][Iv]
                del d2_final[v][Iv]
            else:
                Iv_new = I.closed(param_low, split)
                print("new_iv:", Iv_new)

                d1_final[v][Iv_new] = d1_temp
                d2_final[v][Iv_new] = d2_temp
                # p[v][Iv_new] = u
                #
                # Iv_remaining = I.closed(split, Iv.upper)
                # d1_final[v][Iv_remaining] = d1
                # d2_final[v][Iv_remaining] = d2
                del d1_final[v][Iv]
                del d2_final[v][Iv]

        else:
            if denom > 0:  # needs correction
                Iv_new = I.closed(split, param_high)
                print("new_iv:", Iv_new)

                d1_final[v][Iv_new] = d1_temp
                d2_final[v][Iv_new] = d2_temp
                # p[v][Iv_new] = u
                #
                # Iv_remaining = I.closed(Iv.lower, split)
                # d1_final[v][Iv_remaining] = d1
                # d2_final[v][Iv_remaining] = d2
                del d1_final[v][Iv]
                del d2_final[v][Iv]
            else:
                Iv_new = I.closed(param_low, split)
                print("new_iv:", Iv_new)

                d1_final[v][Iv_new] = d1_temp
                d2_final[v][Iv_new] = d2_temp
                # p[v][Iv_new] = u
                #
                # Iv_remaining = I.closed(split, Iv.upper)
                # d1_final[v][Iv_remaining] = d1
                # d2_final[v][Iv_remaining] = d2
                del d1_final[v][Iv]
                del d2_final[v][Iv]
    print("splits:", split_list)
    return Iv_new, d1_final[v][Iv_new], d2_final[v][Iv_new]

def wfse_constructor(task_case):
    ap = set(['A', 'B', 'C', 'D', 'E', 'O', 'B', 'N'])  # set of atomic propositions
    wfse = Wfse(props=ap, multi=False)
    wfse.init = set()  # HACK

    # add states
    wfse.g.add_nodes_from(['q0', 'q1', 'q2', 'q3', 'q4'])

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
        out_symbol = wfse.bitmap_of_props(set(['A']))
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

        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q1', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute C by A with a penalty 4
        in_symbol = wfse.bitmap_of_props(set(['C']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        weighted_symbols = [(in_symbol, out_symbol, 7)]
        wfse.g.add_edge('q0', 'q2', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q2', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute D by A with a penalty 6
        in_symbol = wfse.bitmap_of_props(set(['D']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        weighted_symbols = [(in_symbol, out_symbol, 11)]
        wfse.g.add_edge('q0', 'q3', attr_dict={'symbols': weighted_symbols})
        weighted_symbols = [(-1, out_symbol, 0)]
        wfse.g.add_edge('q3', 'q0', attr_dict={'symbols': weighted_symbols})

        # Substitute E by A with a penalty 8
        in_symbol = wfse.bitmap_of_props(set(['E']))
        out_symbol = wfse.bitmap_of_props(set(['A']))
        weighted_symbols = [(in_symbol, out_symbol, 9)]
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

    # phi_sdc = '[H^13 !O] ^ [0,15] & [H^2 A] ^ [0,8]'

    # case1_synthesis(phi_sdc, '../data/ts_synthesis.yaml')

    # ts = ts_constructor_TR()
    # print(ts)
    wfse = wfse_constructor(task_case)
    # print(wfse)

    # phi_sdc = '(H^15 !O) & [H^2 A] ^ [0,7]'
    # phi_sdc = '(H^10 !O) & [H^2 A] ^ [0,7]'
    phi_sdc = '[H^1 A] ^ [0,5] & [H^1 P] ^ [0,10]'

    # phi_sdc = '[H^3 A] ^ [0,13] & (H^15 !O)'

    twtl_synthesis(phi_sdc, '../data/sdc_virtual.yaml', wfse)

    # twtl_synthesis(phi_sdc, '../data/test_pareto.yaml', wfse)

    # phi_sdc = '[H^5 C] ^ [0,15] & [H^2 B] ^ [0,30]'

    # twtl_synthesis(phi_sdc, '../data/sdc_case_study.yaml', wfse)

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
