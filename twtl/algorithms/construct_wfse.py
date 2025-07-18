''' This file contains the function to construct the WFSE from the original and relaxed FSA and the penalty value 

 Since both the original and relaxed FSA are DAGs, the resulting product is also a DAG.
''' 

from __future__ import print_function
import itertools as it
from functools import reduce
import operator as op
import logging
from collections import deque
from lomap import Timer
from lomap.classes import Model, Fsa, Wfse
from lomap.algorithms.product import get_default_state_data, get_default_transition_data
import networkx as nx
import matplotlib.pyplot as plt
import time
# from linetimer import CodeTimer
import sys
from itertools import chain
sys.path.append('../')
import twtl


def powerset(iterable):
    '''powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)
    Note: from https://docs.python.org/2.7/library/itertools.html#recipes
    '''
    s = list(iterable)
    return it.chain.from_iterable(it.combinations(s, r)
                                                      for r in range(len(s)+1))



def construct_wfse(fsa_original, fsa_relaxed, penalty): 

    ''' This function constructs the WFSE from the original and relaxed FSA and the penalty value'''

    automata = [fsa_original, fsa_relaxed]

    # set of APs in product
    product_props = set.union(*[set(component.props) for component in automata])

    # Create the product_model
    wfse = Wfse(props=product_props, multi=True)
    # product_model = Wfse(props=product_props, multi=False)

    '''The DFA contains only APs present in the specification and the WFSE contains all the APs in the TS
    So before initializing the product, we need to create a mapping between the APs in the WFSE and the DFA'''


    # ''' The symbol table is able to handle '''
    symbol_tables = []
    for fsa in automata:
        translation_table = dict()
        for fsa_props in powerset(fsa.props):
            fsa_symbol = fsa.bitmap_of_props(fsa_props)
            product_fsa_symbol = wfse.bitmap_of_props(fsa_props)
            other_props = set(wfse.props) - set(fsa.props)

            product_fsa_symbols = set()
            for pfsa_props in powerset(other_props):
                other_pfsa_symbol = wfse.bitmap_of_props(pfsa_props)
                assert not (product_fsa_symbol & other_pfsa_symbol)
                product_fsa_symbols.add(product_fsa_symbol | other_pfsa_symbol)
            translation_table[fsa_symbol] = product_fsa_symbols

        symbol_tables.append(translation_table)

    wfse.init = set()  # Make initial a set
    wfse.final = set()  # Make final a set

    print('symbol tables:', symbol_tables)  

    wfse.final.add(('wfse_virtual_final, fsa_virtual_final'))  ### Add a virtual final state to the product model since multiple final states possible 

    for original_init in fsa_original.init:
        for relaxed_init in fsa_relaxed.init:
            init_state = (original_init, relaxed_init)
            wfse.init.add(init_state)
            wfse.g.add_node(init_state)

    stack = deque(wfse.init)
    # print("stack:", stack)

    while stack:
        print("stack:", stack)
        product_current_state = stack.popleft()
        fsa_original_current, fsa_relaxed_current = product_current_state
        # print("current wfse, current fsa: ", wfse_current, fsa_current)

        if fsa_original_current in fsa_original.final and fsa_relaxed_current in fsa_relaxed.final:
            wfse.final.add(product_current_state)
            wfse.g.add_edge(product_current_state, ('wfse_virtual_final, fsa_virtual_final'), input=[], weight=0, symbols=[])

        ## Iterate over all outgoing edges of WFSE for each input output symbol pair

        for _, fsa_original_next, fsa_original_data in fsa_original.g.out_edges((fsa_original_current), data=True) : 
            for  _, fsa_relaxed_next, fsa_relaxed_data in (fsa_relaxed.g.out_edges((fsa_relaxed_current), data=True)):

                print('fsa original next:', fsa_original_next, 'fsa original data:', fsa_original_data)
                print('fsa relaxed next:', fsa_relaxed_next, 'fsa relaxed data:', fsa_relaxed_data)
    
                for each_sym in fsa_original_data['symbols']:

                    in_sym_bit, out_sym_bit, weight = each_sym
                    print("in_sym_bit, out_sym_bit, weight:", in_sym_bit, out_sym_bit, weight)  
                    in_sym = wfse.symbol_from_bitmap(in_sym_bit)
                    out_sym = wfse.symbol_from_bitmap(out_sym_bit)

                    # print('wfse symbols: ', in_sym, out_sym)

                    wfse_map = symbol_tables[0]

                    if out_sym_bit == -1: 
                        print("*****************************************")
                        print('out sym is -1 ', out_sym_bit, wfse_current)
                        wfse_out_data_mapped = -1

                        fsa_next = fsa_current  
                        next_product_state = (wfse_next, fsa_next)

                        if wfse_next in wfse.final and fsa_next in fsa.final:
                            # print('added to final:', (wfse_next, fsa_next))
                            wfse.final.add((wfse_next, fsa_next))
                            wfse.g.add_edge((wfse_next, fsa_next), ('wfse_virtual_final, fsa_virtual_final'), input=[], weight=0, symbols=[])

                        if next_product_state not in wfse.g.nodes():
                            wfse.g.add_node(next_product_state)
                            wfse.g.add_edge(product_current_state, next_product_state, input=[(in_sym_bit, out_sym_bit, weight)],
                                                        weight=weight, symbols=[(in_sym, out_sym, weight)])
                            
                            stack.append(next_product_state)
                            # print('adding to stack:', next_product_state)

                        elif next_product_state not in wfse.g[product_current_state]:
                            '''node exists; edge doesnt'''
                            wfse.g.add_edge(product_current_state, next_product_state, input=[(in_sym_bit, out_sym_bit, weight)],
                                                        weight=weight, symbols=[(in_sym, out_sym, weight)])

                        else:
                            '''Node already added to product and explored in stack'''
                            if (product_current_state, next_product_state) in wfse.g.edges():

                                ''' edge already added to product; append the inputs and symbols'''
                                wfse.g[product_current_state][next_product_state]['input'].append((in_sym_bit, out_sym_bit, weight))
                                wfse.g[product_current_state][next_product_state]['symbols'].append((in_sym, out_sym, weight))

                                ''''else the edge doent exist; continue'''
                        
                    else:                 
                        wfse_out_data_mapped = wfse_map[out_sym_bit]


                    for _, fsa_next, fsa_data in fsa.g.out_edges(fsa_current, data=True):
                        print('next in fsa', fsa_next, 'with data', fsa_data)
                        fsa_map = symbol_tables[1]
                        # fsa_data_mapped = fsa_map[next(iter(fsa_data['input']))]
                        fsa_data_mapped = []
                        for symbol in fsa_data['input']:
                            fsa_data_mapped.append(list(fsa_map[symbol]))

                        fsa_data_mapped = list(it.chain.from_iterable(fsa_data_mapped))
                        # aux = [set(it.chain.from_iterable(
                        #     [tr[s] for s in fsa.g[u][v]['input']]))
                        #     for u, v, fsa, tr in zip(fsa_current,
                        #                              fsa_next, automata, symbol_tables)]

                        # print('fsa mapping', fsa_data_mapped)
                        # print('out sym', len(out_sym))

                        # print('aux', aux)
                        # bitmaps = set.intersection(*aux)
                        bitmaps = set(fsa_data_mapped).intersection(wfse_out_data_mapped)
                        # print('bitmaps:', bitmaps)

                        print('fsa mapped data:', fsa_data_mapped)
                        print('out sym:', out_sym_bit)

                        # if out_sym_bit == -1 or out_sym_bit is None: 
                        #     print("*****************************************")
                        #     print('out sym is -1 ', out_sym_bit, fsa_current)
                        #     fsa_next = fsa_current 


                        if wfse_out_data_mapped == -1 or out_sym_bit == -1:
                            fsa_next = fsa_current

                        if out_sym_bit in fsa_data_mapped or out_sym_bit == -1:

                        # if wfse_out_data_mapped in fsa_data_mapped:

                            # print('fsa nexts', fsa_next, 'for input', set([out_sym_bit]), 'for current state', fsa_current)

                        # if len(out_sym) and out_sym in fsa_data['guard']:
                        #     print('fsa curent', fsa_current, 'fsa nexts', fsa_next, 'for input', set([out_sym]), 'for current state', fsa_current)

                            next_product_state = (wfse_next, fsa_next)

                            if wfse_next in wfse.final and fsa_next in fsa.final:
                                # print('added to final:', (wfse_next, fsa_next))
                                product_model.final.add((wfse_next, fsa_next))
                                product_model.g.add_edge((wfse_next, fsa_next), ('wfse_virtual_final, fsa_virtual_final'), input=[], weight=0, symbols=[])

                            if next_product_state not in product_model.g.nodes():
                                product_model.g.add_node(next_product_state)
                                product_model.g.add_edge(product_current_state, next_product_state, input=[(in_sym_bit, out_sym_bit, weight)],
                                                         weight=weight, symbols=[(in_sym, out_sym, weight)])
                                
                                stack.append(next_product_state)
                                # print('adding to stack:', next_product_state)

                            elif next_product_state not in product_model.g[product_current_state]:
                                '''node exists; edge doesnt'''
                                product_model.g.add_edge(product_current_state, next_product_state, input=[(in_sym_bit, out_sym_bit, weight)],
                                                         weight=weight, symbols=[(in_sym, out_sym, weight)])

                            else:
                                '''Node already added to product and explored in stack'''
                                if (product_current_state, next_product_state) in product_model.g.edges():

                                    ''' edge already added to product; append the inputs and symbols'''
                                    product_model.g[product_current_state][next_product_state]['input'].append((in_sym_bit, out_sym_bit, weight))
                                    product_model.g[product_current_state][next_product_state]['symbols'].append((in_sym, out_sym, weight))

                                    ''''else the edge doent exist; continue'''


                                
    return wfse


def main():

    formula = '[H^0 A]^ [0,2]'
    _, dfa_1, dfa_inf, bdd = twtl.translate(formula, kind='both', norm=True)


    formula = '[H^0 A ]^ [0,2] | [H^0 B]^[0,2]'
    _, dfa_2, dfa_inf, bdd = twtl.translate(formula, kind='both', norm=True)

    wfse = construct_wfse(dfa_1, dfa_2, 5) 
    print("wfse: ", wfse)

if __name__ == '__main__':
    main()