license_text='''
    Module implements methods for automata-based control policy synthesis from
    TWTL formulae.
    Copyright (C) 2015-2016  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab,
    Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
'''
.. module:: synthesis.py
   :synopsis: Module implements methods for automata-based control policy
   synthesis from TWTL formulae.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

import logging
import itertools as it
from operator import attrgetter

import numpy as np
import networkx as nx

from lomap import Ts, Model
from lomap import ts_times_ts
from dfa import DFAType, Op

__all__ = ['ts_times_fsa', 'ts_times_ts', 'expand_duration_ts',
    'one_loop_reach_graph', 'policy_output_word', 'simple_control_policy',
    'partial_control_policies', 'relaxed_control_policy',
    'compute_control_policy']

ninf = float('-Inf')
ts_times_ts

''' Comments on implementation:
1) Finding the single source shortest paths in a DAG has O(V+E) complexity
(both positive and negative weights are allowed).
2) FUTURE: find best data structure with operations: union, intersection and
concatenation with update.
3) FUTURE: better way to represent paths and test for equality and compare.
4) FUTURE: Possible solution is to define a total order on the paths and
maintain the path sets as sorted lists.
5) DAG x G = DAG; Proof by contradiction. Only applicable to normal dfa, not
the infinity version.
'''

class ControlPath(object):
    '''Class defining a (partial) control policy as a path in a product
    automaton with its associated optimal temporal relaxation value `tau`.
    '''

    __slots__ = ['path', 'tau', '_hash']

    def __init__(self, path=None, tau=ninf):
        self.path = tuple(path)
        self.tau = tau
        self._hash = hash(self.path)

    def update_hash(self):
        self._hash = hash(self.path)

    def __eq__(self, cpath):
        # using the stored hash value first saves some time when comparing paths
        if self._hash != cpath._hash:
            return False
        return self.path == cpath.path

    def __str__(self):
        return '|| {} >> : {}'.format(self.tau, self.path)
    __repr__ = __str__

class ControlPathsSet(object):
    '''Class defining a collection of distinct control paths. The data structure
    support three basic operations: union, intersection and concatenation which
    also update the temporal relaxation values of the control paths.
    '''

    def __init__(self, paths=None, copy=False):
        '''Construct a new set of control paths from the given list of paths.
        If `copy` is true, then a (shallow) copy of the list is made, otherwise
        the provided list is used.
        Note: The `copy` parameter might be removed in future versions.
        '''
        if paths is None:
            self.paths = []
        elif copy:
            self.paths = list(paths)
        else:
            self.paths = paths

    def union(self, other):
        '''Implements the union of two sets of control paths. If a path is in
        both sets, then it is added once and its `tau` value is set as the
        minimum between the two instances.
        '''
        M_left, M_right = self, other
        M = ControlPathsSet(M_left.paths, copy=True)
        for pr in M_right:
            for pl in M_left:
                if pl == pr:
                    pl.tau = min(pl.tau, pr.tr)
                    break
            else:
                M.paths.append(pr)
        return M

    def intersection(self, other):
        '''Implements the intersection of two sets of control paths. The `tau`
        value of a path in the intersection is set as the maximum between the
        two instances.
        '''
        M_left, M_right = self, other
        M = ControlPathsSet()
        for pl in M_left:
            for pr in M_right:
                if pl == pr:
                    M.paths.append(pl)
                    pl.tau = max(pl.tau, pr.tau)
                    break
        return M

    def concatenate(self, other):
        '''Implements the pairwise concatenation of paths from the current set
        with compatible paths from the `other` set. An ordered pair of path is
        compatible if the first path ends in the same state as the second path.
        The concatenation of the paths does not repeat the common state.
        Finally, the `tau` value of the a resulting path is set as the maximum
        between the `tau` value of the first path and the `tau` value of the
        second one minus one, which accounts for the non-repetition of the
        common state.
        '''
        M_left, M_right = self, other
        M = ControlPathsSet()
        for pl in M_left:
            for pr in M_right:
                if pl.path[-1] == pr.path[0]:
                    logging.debug('[Cat] accept %s + %s ; max(%s, %s)',
                                  pl, pr, pl.tau, pr.tau)
                    M.paths.append(ControlPath(pl.path[:-1] + pr.path,
                                               tau=max(pl.tau, pr.tau-1)))
        return M

    __or__ = union
    __and__ = intersection
    __add__ = concatenate

    def __iter__(self):
        return iter(self.paths)


def ts_times_fsa(ts, fsa): #FIXME: product automaton convention
    # Create the product_model
    product_model = Model(directed=True, multi=False)

    init_state = (ts.init.keys()[0], fsa.init.keys()[0])
    product_model.init[init_state] = 1
    product_model.g.add_node(init_state)
    if init_state[1] in fsa.final:
        product_model.final.add(init_state)

    stack = [init_state]
    # Consume the stack
    while(stack):
        cur_state = stack.pop()
        ts_state, fsa_state = cur_state

        for ts_next in ts.next_states_of_wts(ts_state, traveling_states = False):
            ts_next_state = ts_next[0]
            ts_prop = ts.g.node[ts_next_state].get('prop',set())

            for fsa_next_state in fsa.next_states_of_fsa(fsa_state, ts_prop):
                next_state = (ts_next_state, fsa_next_state)

                if(next_state not in product_model.g):
                    # Add the new state
                    product_model.g.add_node(next_state)
                    # Add transition w/ weight
                    product_model.g.add_edge(cur_state, next_state)
                    # Mark as final if final in fsa
                    if fsa_next_state in fsa.final:
                        product_model.final.add(next_state)
                    # Continue search from next state
                    stack.append(next_state)

                elif(next_state not in product_model.g[cur_state]):
                    product_model.g.add_edge(cur_state, next_state)
    return product_model

def expand_duration_ts(ts):
    '''Expands the edges of the duration transition system such that all edges
    in the new transition system have duration one.
    '''
    assert not ts.multi
    ets = Ts(directed=ts.directed, multi=False)

    # copy attributes
    ets.name = ts.name
    ets.init = ts.init

    # copy nodes with data
    ets.g.add_nodes_from(ts.g.nodes_iter(data=True))

    # expand edges
    ets.state_map = dict() # reverse lookup dictionary for intermediate nodes
    ng = it.count()
    for u, v, data in ts.g.edges_iter(data=True):
        # generate intermediate nodes
        aux_nodes = [u] + [ng.next() for _ in range(data['duration']-1)] + [v]
        u_pos = np.array(ts.g.node[u]['position'])
        v_pos = np.array(ts.g.node[v]['position'])
        aux_pos = [{'position' : tuple(u_pos + s * (v_pos - u_pos)), 's': s}
                                 for s in np.linspace(0, 1, num=len(aux_nodes))]
        ets.g.add_nodes_from(zip(aux_nodes, aux_pos))
        # create intermediate edges
        edge_list = zip(aux_nodes[:-1], aux_nodes[1:]) + zip(aux_nodes, aux_nodes)
        # add intermediate edges
        ets.g.add_edges_from(edge_list, weight=1)
        # update reverse lookup map
        for state in aux_nodes[1:-1]:
            ets.state_map[state] = (u, v)

    if logging.getLogger().isEnabledFor(logging.DEBUG):
        logging.debug('[extend_ts] TS: ({}, {}) ETS:({}, {})'.format(
                        nx.number_of_nodes(ts.g), nx.number_of_edges(ts.g),
                        nx.number_of_nodes(ets.g), nx.number_of_edges(ets.g)))
    return ets

def one_loop_reach_graph(pa, states=None):
    '''Computes the on-loop reachability graph for persistent missions.'''
    if not states:
        states = set([x for x, _ in pa.final])
    s_init = pa.init.keys()[0][1]

    g = nx.DiGraph(name='One loop reachability graph')
    for x in states:
        paths = nx.shortest_path_length(pa.g, source=(x, s_init))
        edges = [(x, p[0], d) for p, d in paths.iteritems() if p in pa.final]
        g.add_weighted_edges_from(edges)

    return g

def policy_output_word(path_ts, ap):
    '''Returns the output word corresponding to the given path in a transition
    system. The underlying transition system corresponds to a single robot or to
    multiple ones, in which case the states of the path are assumed to be
    tuples.
    '''
    if isinstance(path_ts[0], tuple):
        output_word = [set(x) & ap for x in path_ts]
    else:
        output_word = [set([x]) & ap for x in path_ts]
    return output_word

def simple_control_policy(pa):
    '''Computes a control policy which minimizes the total length (makespan)
    of the policy. It can be used on product automata obtained from both normal
    and infinity specification FSAs. In the infinity automata case, the returned
    policy corresponds to a valid relaxation, but it may not in general provide
    the best temporal relaxation.
    '''
    if not pa.final:
        return None
    # add virtual node with incoming edges from all final states
    pa.g.add_edges_from([(p, 'virtual') for p in pa.final])
    # compute optimal path in PA and then project onto the TS
    pa_path = nx.shortest_path(pa.g, source=pa.init.keys()[0], target='virtual')
    assert pa_path[-2] in pa.final
    pa.g.remove_node('virtual')
    return [x for x, _ in pa_path[:-1]]

def partial_control_policies(pa, dfa, init, finish, constraint=None):
    '''Computes all partial optimal control policies between the set of initial
    states ``init'' and the set of final states ``finish''. The paths are
    computed on the product automaton ``pa'' and are optimal with respect to
    path length. Moreover, if the set of edges ``constraint'' is given, then
    only paths (projected onto ``dfa'') ending in these edges are returned.
    Finally, paths which are intersect the set of finals states more than once
    are pruned.
    '''
    #TODO: return path lengths as well if requested
    logging.debug('[PartialControl] init: %s, final: %s, constraint: %s',
                  init, finish, constraint)
    sat_paths = []
    for state in (p for p in pa.g.nodes_iter() if p[1] in init):
        paths = nx.shortest_path(pa.g, source=state)
        if constraint is None:
            sat_paths.extend([path for p, path in paths.iteritems()
                                       if p[1] in finish])
        else:
            logging.debug('[]')
            sat_paths.extend([path for p, path in paths.iteritems()
                if p[1] in finish and # is final
                path[-2][1] in constraint.keys() and # is in restriction
                dfa.g.has_edge(path[-2][1], p[1]) and # there is an edge
                # the edge activates properly
                dfa.g[path[-2][1]][p[1]]['input'] <= constraint[path[-2][1]]])
    assert len(sat_paths) == len(set(map(tuple, sat_paths)))

    # prune paths which intersect the set of final states more than once
    sat_paths_aux = []
    for path in sat_paths:
        if not [p for p in path[:-1] if p[1] in finish]:
            sat_paths_aux.append(path)
    sat_paths = sat_paths_aux

    return sat_paths

def partial_control_policies2(pa, dfa, init, finish, constraint=None):
    '''An optimized version of ``partial_control_policies'' based on
    breath-first search traversal of the graph associated with ``pa''. It also,
    returns paths lengths.
    Note: It is still experimental and needs testing.
    '''
    logging.debug('[PartialControl] init: %s, final: %s, constraint: %s',
                  init, finish, constraint)
    if constraint is not None:
        C = constraint.viewkeys()

    sat_paths = []
    for source in (p for p in pa.g.nodes_iter() if p[1] in init):
        level=1                  # the current level
        nextlevel={source:1}     # list of nodes to check at next level
        paths={source:[source]}  # paths dictionary  (paths to key from source)
        lengths={source:0}
        while nextlevel:
            thislevel=nextlevel
            nextlevel={}
            for v in thislevel:
                if v[1] not in finish: #if not a final state
                    for w in pa.g[v]:
                        if w not in paths:
                            paths[w]=paths[v]+[w]
                            lengths[w] = level
                            nextlevel[w]=1
            level=level+1
        if constraint is None:
            sat_paths.extend([(paths[p], lengths[p]) for p in paths
                                                         if p[1] in finish])
        else:
            sat_paths.extend([(path, lengths[p]) for p, path in paths.iteritems()
               if p[1] in finish and # is final
                  path[-2][1] in C and # is in restriction
                  # the edge activates properly
                  dfa.g[path[-2][1]][p[1]]['input'] <= constraint[path[-2][1]]])

    return sat_paths

def relaxed_control_policy(tree, dfa, pa, constraint=None):
    '''Computes a control policy with minimum maximum temporal relaxation. It
    also returns the value of the optimal relaxation.
    '''
    assert tree.wdf

    if tree.unr: # primitive/unrelaxable formula
        paths = partial_control_policies(pa, dfa, tree.init, tree.final, constraint)
        return ControlPathsSet([ControlPath(p) for p in paths])

    if tree.wwf and tree.operation == Op.event: # leaf within operator
        paths = partial_control_policies(pa, dfa, tree.init, tree.final, constraint)
        return ControlPathsSet([ControlPath(path, len(path) - tree.high - 1)
                                    for path in paths])

    if not tree.wwf and tree.operation == Op.event:
        M_ch = relaxed_control_policy(tree.left, dfa, pa, constraint)
        if tree.low == 0:
            for cpath in M_ch:
                cpath.tau = max(len(cpath.path) - tree.high - 1, cpath.tau)
            return M_ch

        M = ControlPathsSet()
        for cp in M_ch:
            paths = nx.shortest_path(pa.g, target=cp.path[0])
            sat_paths = [p[:-1]+cp.path for p_i, p in paths.iteritems()
                                         if p_i in tree.init]
            tau = max(len(cp.path)+tree.low-tree.high, cp.tau) #TODO: should I subtract -1?
            M.paths.extend([ControlPath(p, tau) for p in sat_paths])
        return M

    if tree.operation == Op.cat:
        M_left = relaxed_control_policy(tree.left, dfa, pa)
        M_right = relaxed_control_policy(tree.right, dfa, pa, constraint)
        # concatenate paths from M_left with paths from M_rigth
        M = M_left + M_right
        return M

    if tree.operation == Op.intersection:
        M_left = relaxed_control_policy(tree.left, dfa, pa, constraint)
        M_right = relaxed_control_policy(tree.right, dfa, pa, constraint)
        # intersection of M_left and M_rigth
        M = M_left & M_right
        return M

    if tree.operation == Op.union:
        if constraint is None:
            c_left = {s: ch.both | ch.left for s, ch in tree.choices.iteritems()}
            c_right = {s: ch.both | ch.right for s, ch in tree.choices.iteritems()}
        else:
            c_left = dict()
            c_right = dict()
            for s in tree.choices.viewkeys() & constraint.viewkeys():
                c_left[s] = constraint[s] & (tree.choices[s].both | tree.choices[s].left)
                c_right[s] = constraint[s] & (tree.choices[s].both | tree.choices[s].right)

        M_left = relaxed_control_policy(tree.left, dfa, pa, c_left)
        M_right = relaxed_control_policy(tree.right, dfa, pa, c_right)
        # union of M_left and M_rigth
        M = M_left | M_right
        return M

    raise ValueError('Unknown operation: {}!'.format(tree.operation))

def compute_control_policy(pa, dfa, kind):
    '''Computes a control policy from product automaton pa. It also returns the
    corresponding output word over the set of atomic propositions ap. If there
    is no satisfying trajectory, then the control policy and the output word are
    both None. The last returned value is the maximum temporal relaxation. If a
    normal automaton is provided then the returned value is None.
    '''
    if kind == DFAType.Normal:
        optimal_ts_path = simple_control_policy(pa)
        optimal_tau = None
    elif kind == DFAType.Infinity:
        policies = relaxed_control_policy(dfa.tree, dfa, pa)
        if not policies:
            return None, None, None
        # keep only policies which start from the initial PA state
        policies.paths = [p for p in policies if p.path[0] in pa.init.keys()]
        # choose optimal policy with respect to temporal robustness
        optimal_pa_path = min(policies, key=attrgetter('tau'))
        optimal_ts_path = [x for x, _ in optimal_pa_path.path]
        optimal_tau = optimal_pa_path.tau
    else:
        raise ValueError('Invalid value for the type of automata construction!')
    if optimal_ts_path is None:
        return None, None, None
    output_word = policy_output_word(optimal_ts_path, set(dfa.props.keys()))
    return optimal_ts_path, output_word, optimal_tau

def verify(ts, dfa):
    '''Verifies if all trajectories of a transition system satisfy a temporal
    relaxation of a TWTL formula. The function takes as input the transition
    system and the annotated DFA corresponding to the TWTL formula. A trap state
    is added to the DFA and then the product automaton between the two models is
    computed. The return value is a Boolean specifying if the desired property
    holds.
    '''
    assert dfa.kind == DFAType.Infinity

    dfa_complete = dfa.clone()
    dfa_complete.add_trap_state()
    dfa_complete.g.remove_edge(iter(dfa_complete.final).next(), 'trap')

    logging.info('Constructing product automaton with infinity DFA!')
    pa = ts_times_fsa(ts, dfa_complete)
    logging.info('Product automaton size is: (%d, %d)', *pa.size())

    return nx.is_directed_acyclic_graph(pa.g) and pa.final and \
            all([pa.g.out_degree(u) > 0 for u in pa.g.nodes_iter()
                                            if u not in pa.final])

if __name__ == '__main__':
    pass
