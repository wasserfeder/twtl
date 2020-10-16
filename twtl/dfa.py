license_text='''
    Module implements Time-Window Temporal Logic operations.
    Copyright (C) 2015-2019  Cristian Ioan Vasile <cvasile@bu.edu>
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
.. module:: dfa.py
   :synopsis: Module implements Time-Window Temporal Logic operations.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

import logging
logger = logging.getLogger(__name__)
import itertools as it
from StringIO import StringIO

import networkx as nx

from lomap import Fsa


'''
FUTURE:
1) AST tree rewriting from general form to within-disjunction free (WDF) form.
2) Minimization of DFAs at each step of the procedure for both DFA versions,
normal and infinity.
3) Optimization: disjoint satisfaction of atomic propositions.
4) Special type of complementation.

Assumptions: The following code assumes that the given TWTL formula:
1) has no ambiguous concatenation;
2) negations only in front of atomic propositions;
3) is in WDF form (will be removed in the future).
'''


class DFAType(object):
    '''Class defining the two DFA types, normal DFA and infinity DFA.'''
    Normal, Infinity = range(2)
'''The DFA type to generate.'''
dfa_type = DFAType.Infinity

def setDFAType(val):
    '''Sets the DFA generation method.'''
    global dfa_type
    if val not in (DFAType.Normal, DFAType.Infinity):
        raise ValueError()
    dfa_type = val
def getDFAType():
    '''Retrieves the DFA generation method.'''
    global dfa_type
    return dfa_type

optimize = True
def setOptimizationFlag(val=True):
    '''Sets the optimization flag'''
    global optimize
    optimize = bool(val)
def getOptimizationFlag():
    '''Retrieves the optimization flag.'''
    global optimize
    return optimize

class Op(object):
    '''Class defining the operation codes for the TWTL operators.'''
    nop, accept, hold, neg, cat, intersection, union, within, event = range(9)
    operations = (nop, accept, hold, neg, cat, intersection, union, within, event)
    names = ['NoOperation', 'AcceptProp', 'Hold', 'Negation',
             'Concatenation', 'Intersection', 'Union', 'Within', 'Eventually']
    @classmethod
    def str(cls, op):
        if op in cls.operations:
            return cls.names[op]
        raise ValueError('Unknown operation!')


class Choice(object):
    '''Class defining the choices for disjunction operators.'''
    __slots__ = ['both', 'left', 'right']

    def __init__(self, both=None, left=None, right=None):
        self.both = set(both) if both is not None else set()
        self.left = set(left) if left is not None else set()
        self.right = set(right) if right is not None else set()

    def __repr__(self): return self.__str__()
    def __str__(self):
        return 'both={} left={} right={}'.format(self.both, self.left,
                                                 self.right)


class DFATreeNode(object):
    '''Class defining a tree node to store information about TWTL operators
    used to compute relaxed control policies with respect to deadlines.
    Each node stores the associated operation, the initial and final states of
    the automaton corresponding to the formula associated with the tree, the
    left and right subtrees, some flags used for sanity checks and additional
    data specific to the associated operation.
    '''
    def __init__(self, operation=Op.nop, left=None, right=None,
                 init = None, final = None,
                 wdf=True, wwf=True, unr=True, ndj=0, **kwargs):
        self.operation = operation
        self.left = left
        self.right = right
        self.wdf = wdf # within-disjunction free
        self.wwf = wwf # within-within free
        self.unr = unr # un-relaxable, i.e. within free
        # number of disjunction operators
        self.ndj = ndj + (self.operation == Op.union)
        self.init = set(init)
        self.final = set(final)

        if self.operation == Op.event:
            self.low = kwargs.get('low', 0)
            self.high = kwargs.get('high', 0)
        elif self.operation == Op.union:
            self.choices = kwargs.get('choices', None)

    def relabel(self, mapping, expand=False):
        '''Relabels the data about the DFA states which are stored within the
        nodes of the tree. The map `mapping` is used to translate the states'
        labels. If `expand` is set, then the `mapping` is treated as a
        multi-valued map and each state is replicated as needed.
        '''
        stack = [self]
        while stack:
            tree = stack.pop()
            if expand:
                logging.debug('state: %s \n init: %s\n final: %s',
                              Op.str(tree.operation),
                              list(*it.chain([mapping[u] for u in tree.init])),
                              list(*it.chain([mapping[u] for u in tree.final])))
                tree.init = set(*it.chain([mapping[u] for u in tree.init]))
                tree.final = set(*it.chain([mapping[u] for u in tree.final]))
                if tree.operation == Op.union:
                    tree.choices = dict([(key, v)
                                            for k, v in tree.choices.iteritems()
                                                for key in mapping[k]])
            else:
                tree.init = set([mapping[u] for u in tree.init])
                tree.final = set([mapping[u] for u in tree.final])
                if tree.operation == Op.union:
                    tree.choices = dict([(mapping[k], v)
                                          for k, v in tree.choices.iteritems()])
            if tree.right is not None:
                stack.append(tree.right)
            if tree.left is not None:
                stack.append(tree.left)

    def normalize(self, init, final):
        '''Resets the initial and final sets of states in the tree such that it
        eliminates unreachable start or end configurations, i.e. (1) a
        sub-formula must start and end at the same states as the formula it is
        part of in the case of disjunction, conjunction, hold, accept and
        eventually operators; and (2) in case of concatenation, the left
        sub-formula must start at the same states as the larger formula, while
        the right sub-formula must end at same final states as the larger
        formula.
        '''
#         return
        if self.operation == Op.cat:
            if init is not None:
                self.init = set(init)
                self.left.normalize(init, None)
            if final is not None:
                self.final = set(final)
                self.right.normalize(None, final)
        else:
            if init is not None:
                self.init = set(init)
            if final is not None:
                self.final = set(final)
            if init is not None or final is not None:
                if self.left is not None:
                    self.left.normalize(init, final)
                if self.right is not None:
                    self.right.normalize(init, final)

    def pprint(self, level=0, indent=2):
        '''Returns a multi-line string representation of the whole tree.'''
        ret = StringIO()
        print>>ret, ' '*(level*indent), str(self)
        print>>ret, ' '*(level*indent), 'Init:', self.init
        print>>ret, ' '*(level*indent), 'Final:', self.final
        if self.operation == Op.union:
            print>>ret, ' '*(level*indent), 'Choices:'
            for k, v in self.choices.iteritems():
                print>>ret, ' '*((level+1)*indent), k, '->', v
        if self.left is not None:
            print>>ret, ' '*(level*indent), 'Left:'
            print>>ret, self.left.pprint(level=level+1),
        if self.right is not None:
            print>>ret, ' '*(level*indent), 'Right:'
            print>>ret, self.right.pprint(level=level+1),
        ret_str = str(ret.getvalue())
        ret.close()
        return ret_str

    def __str__(self):
        return 'Op: {} Flags[WDF, WWF, UNR]: {} {} {}'.\
                format(Op.str(self.operation), self.wdf, self.wwf, self.unr)


def copy_tree(dfa_src, dfa_dest, mapping=None):
    '''Copies the tree from the source to the destination automaton and
    translates the tree data using the mapping dictionary.
    '''
    if getDFAType() == DFAType.Infinity:
        dfa_dest.tree = dfa_src.tree
        if mapping is not None:
            dfa_dest.tree.relabel(mapping)

def init_tree(dfa, operation=Op.nop):
    '''Creates a new AST tree node and adds it to the automaton `dfa`. The
    operation corresponding to the tree node is given by the `operation`
    parameter (default=`Op.nop`).
    '''
    if DFAType.Infinity:
        assert operation in Op.operations
        dfa.tree = DFATreeNode(operation, init=dfa.init.keys(), final=dfa.final)

def mark_eventually(dfa_src, dfa_dest, low, high):
    '''Creates a new AST tree node corresponding to a within operator and adds
    it to the destination automaton `dfa_dest`. The child subtree is copied from
    the source automaton `dfa_src`.
    '''
    # create new AST tree
    dfa_dest.tree = DFATreeNode(Op.event, left=dfa_src.tree, right=None,
                                init=dfa_dest.init.keys(), final=dfa_dest.final,
                                low=low, high=high)
    # update flags
    dfa_dest.tree.wdf = (dfa_src.tree.ndj == 0)
    assert dfa_dest.tree.wdf, 'Need within-disjunction free form!'
    dfa_dest.tree.wwf = dfa_src.tree.unr
    dfa_dest.tree.unr = False
    dfa_dest.tree.ndj = dfa_src.tree.ndj

def mark_concatenation(dfa_src1, dfa_src2, dfa_dest):
    '''Creates a new AST tree node corresponding to a concatenation operator and
    adds it to the destination automaton `dfa_dest`. The children subtrees are
    copied from the source automata `dfa_src1` and `dfa_src2`.
    '''
    # create new AST tree
    dfa_dest.tree = DFATreeNode(Op.cat, left=dfa_src1.tree, right=dfa_src2.tree,
                                init=dfa_dest.init.keys(), final=dfa_dest.final)
    # update flags
    dfa_dest.tree.wdf = dfa_src1.tree.wdf and dfa_src2.tree.wdf
    dfa_dest.tree.wwf = dfa_src1.tree.wwf and dfa_src2.tree.wwf
    dfa_dest.tree.unr = dfa_src1.tree.unr and dfa_src2.tree.unr
    dfa_dest.tree.ndj = dfa_src1.tree.ndj + dfa_src2.tree.ndj

def mark_product(dfa_src1, dfa_src2, dfa_dest, operation, choices=None):
    '''Creates a new AST tree node corresponding to a disjunction or conjunction
    operator and adds it to the destination automaton `dfa_dest`. The children
    subtrees are copied from the source automata `dfa_src1` and `dfa_src2`.
    '''
    final_dest = iter(dfa_dest.final).next()
    final_src1 = iter(dfa_src1.final).next()
    final_src2 = iter(dfa_src2.final).next()
    # relabel data in left tree
    mapping = dict([(u, []) for u in dfa_src1.g.nodes_iter()])
    for u, v in dfa_dest.g.nodes_iter():
        mapping[u].append((u, v))
    assert final_dest in mapping[final_src1]
    dfa_src1.tree.relabel(mapping, expand=True)
    # relabel data in right tree
    mapping = dict([(v, []) for v in dfa_src2.g.nodes_iter()])
    for u, v in dfa_dest.g.nodes_iter():
        mapping[v].append((u, v))
    assert final_dest in mapping[final_src2]
    dfa_src2.tree.relabel(mapping, expand=True)
    # create new AST tree
    dfa_dest.tree = DFATreeNode(operation, left=dfa_src1.tree,
                                right=dfa_src2.tree,
                                init=dfa_dest.init.keys(), final=dfa_dest.final,
                                choices=choices)
    # update init and final states of AST nodes
    if getOptimizationFlag():
        dfa_dest.tree.normalize(init=dfa_dest.init.keys(), final=dfa_dest.final)
    # update flags
    dfa_dest.tree.wdf = dfa_src1.tree.wdf and dfa_src2.tree.wdf
    dfa_dest.tree.wwf = dfa_src1.tree.wwf and dfa_src2.tree.wwf
    dfa_dest.tree.unr = dfa_src1.tree.unr and dfa_src2.tree.unr
    dfa_dest.tree.ndj = dfa_src1.tree.ndj + dfa_src2.tree.ndj\
                        + (operation == Op.union)


def relabel_dfa(dfa, mapping='default', start=0, copy=False):
    '''Relabels the DFA. The new labels are given by the mapping dictionary. By
    default, it relabels the states with integers with the lowest one given by
    start. The dictionary can be a partial mapping of the nodes. The states
    which are not specified are labeled with integers starting from start.
    If copy is True a new copy of the DFA is returned, otherwise it performs an
    in-place relabeling.
    '''
    if mapping is 'default': # default mapping
        mapping = dict()
    keys = mapping.keys()
    nodes = [u for u in dfa.g.nodes_iter() if u not in keys]
    mapping.update(dict(zip(nodes, it.count(start))))

    if copy: # create new dfa
        ret = Fsa(dfa.name, dfa.props, dfa.multi)
    else: # in-place relabeling
        ret = dfa
    # relabel state, inital state and set of final states
    ret.g = nx.relabel_nodes(dfa.g, mapping=mapping, copy=True)
    ret.init = dict([(mapping[u], 1) for u in dfa.init.keys()])
    ret.final = set([mapping[u] for u in dfa.final])
    # copy tree
    copy_tree(dfa, ret, mapping=mapping)
    return ret



def minimize_dfa(dfa):
    '''for now it does nothing :)
    # FUTURE: implement FSA minimization w/ lomap
    # FUTURE: implement DFCA minimization w/ lomap
    # Korner algorithm or Incremental algorithm by Campeanu, Paun
    '''
    return dfa



def accept_prop(props, prop=None, boolean=None):
    '''Creates a DFA which accepts:
    1) all symbols which contain proposition prop, if prop is not None;
    2) all symbols, if boolean is True;
    3) no symbol, if boolean is False.
    '''
    if prop is not None:
        assert prop in props
        guard = prop
        name = '(Prop ' + str(prop) + ')'
        logger.debug('[accent_prop] Prop: %s Props: %s', prop, props)
    elif boolean is not None:
        assert type(boolean) == bool
        guard = '(1)' if boolean else '(0)'
        name = '(Bool ' + str(boolean) + ')'
        logger.debug('[accent_prop] Boolean: %s Props: %s', boolean, props)
    else:
        raise AssertionError('Either prop or boolean must be given!')

    dfa = Fsa(name, props, multi=False)
    dfa.name = name
    bitmaps = dfa.get_guard_bitmap(guard)
    ngen = it.count()
    u, v = ngen.next(), ngen.next()
    dfa.g.add_edge(u, v, attr_dict={'weight': 0, 'input': bitmaps,
                                    'guard' : guard, 'label': guard})
    dfa.init[u] = 1
    dfa.final.add(v)

    init_tree(dfa, operation=Op.accept)
    return dfa

def hold(props, prop, duration, negation=False):
    '''Creates a DFA which accepts a sequence of symbols all containing
    proposition prop. The length of the sequence is duration+1 corresponding to
    duration time intervals. If negation is True, then the symbols must not
    contain prop instead.
    '''
    assert prop in props

    guard = prop if not negation else '!' + prop
    name = '(Hold {} {}{} )'.format(duration, 'not ' if negation else '', prop)
    dfa = Fsa(name, props, multi=False)
    bitmaps = dfa.get_guard_bitmap(guard)

    ngen = it.count()
    nodes = [ngen.next() for _ in range(duration+2)]
    attr_dict={'weight': 0, 'input': bitmaps, 'guard' : guard, 'label': guard}
    dfa.g.add_path(nodes, **attr_dict)

    u, v = nodes[0], nodes[-1]
    dfa.init[u] = 1
    dfa.final.add(v)

    init_tree(dfa, operation=Op.hold)
    logger.debug('[hold] Prop: %s Duration: %d Negation: %s Props: %s', prop,
                 duration, negation, props)
    return dfa

def complement(dfa):
    '''Negation (complementation) is not supported for TWTL formulae in the
    current implementation, since we are using the assumption that automata have
    only one final state. This assumption does not hold under the normal
    definition of complementation. In the future, we are going to introduce a
    special complement operation which preserves this property and is tailored
    for specifications with time-windows.
    '''
    raise NotImplementedError

def concatenation(dfa1, dfa2):
    '''Creates a DFA which accepts the language of concatenated word accepted by
    dfa1 and dfa2. Is assumes that concatenation is non-ambiguous, i.e. every
    word in the resulting language can be uniquely decomposed into a prefix word
    from the language accepted by dfa1 and a suffix word from the language
    accepted by dfa2.
    Theorem: card(lang(concatenate(dfa1, dfa2))) ==
                                             card(lang(dfa1)) x card(lang(dfa2))
    '''
    assert dfa1.directed == dfa2.directed and dfa1.multi == dfa2.multi
    assert dfa1.props == dfa2.props
    assert dfa1.alphabet == dfa2.alphabet
    assert len(dfa1.init) == 1 and len(dfa2.init) == 1
    assert len(dfa1.final) == 1 and len(dfa2.final) == 1

    name = '(Concat {} {} )'.format(dfa1.name, dfa2.name)
    dfa = Fsa(name, dfa1.props, dfa1.multi)

    # relabel the two DFAs to avoid state name collisions and merge the final
    # state of dfa1 with the initial state of dfa2
    relabel_dfa(dfa1, start=0)
    init2 = dfa2.init.keys()[0]
    final1 = iter(dfa1.final).next()
    relabel_dfa(dfa2, mapping={init2: final1}, start=dfa1.g.number_of_nodes())
    assert len(set(dfa1.g.nodes()) & set(dfa2.g.nodes())) == 1
    dfa.g.add_edges_from(dfa1.g.edges_iter(data=True))
    dfa.g.add_edges_from(dfa2.g.edges_iter(data=True))

    # define initial state and final state
    dfa.init = dict(dfa1.init)
    dfa.final = set(dfa2.final)

    if getDFAType() == DFAType.Infinity:
        mark_concatenation(dfa1, dfa2, dfa)
    elif getDFAType() == DFAType.Normal:
        # minimize the DFA
        dfa = minimize_dfa(dfa)
    else:
        raise ValueError('DFA type must be either DFAType.Normal or ' +
                         'DFAType.Infinity! {} was given!'.format(getDFAType()))

    logger.debug('[concatenation] DFA1: %s DFA2: %s', dfa1.name, dfa2.name)
    return dfa

def intersection(dfa1, dfa2):
    '''Creates a DFA which accepts the intersection of the languages
    corresponding to the two DFAs. The conjunction operation of TWTL is mapped
    to intersection.
    If an infinity DFA is generated, the corresponding meta-data is copied as
    well.
    '''
    assert dfa1.directed == dfa2.directed and dfa1.multi == dfa2.multi
    assert dfa1.props == dfa2.props
    assert dfa1.alphabet == dfa2.alphabet
    assert len(dfa1.init) == 1 and len(dfa2.init) == 1
    assert len(dfa1.final) == 1 and len(dfa2.final) == 1

    name = '(Intersection {} {} )'.format(dfa1.name, dfa2.name)
    dfa = Fsa(name, dfa1.props, dfa1.multi)

    init = list(it.product(dfa1.init.keys(), dfa2.init.keys()))
    dfa.init = dict(zip(init, (1,)*len(init)))
    assert len(dfa.init) == 1

    stack = list(init)
    while stack:
        u1, u2 = stack.pop()
        for _, v1, d1 in dfa1.g.edges_iter(u1, data=True):
            for _, v2, d2 in dfa2.g.edges_iter(u2, data=True):
                bitmaps = d1['input'] & d2['input']
                if bitmaps:
                    if (v1, v2) not in dfa.g:
                        stack.append((v1, v2))
                    guard = '({}) & ({})'.format(d1['guard'], d2['guard'])
                    dfa.g.add_edge((u1, u2), (v1, v2),
                                   attr_dict={'weight': 0, 'input': bitmaps,
                                              'guard' : guard, 'label': guard})
        if u1 in dfa1.final:
            for _, v2, d2 in dfa2.g.edges_iter(u2, data=True):
                if (u1, v2) not in dfa.g:
                    stack.append((u1, v2))
                bitmaps = set(d2['input'])
                guard = d2['guard']
                dfa.g.add_edge((u1, u2), (u1, v2),
                               attr_dict={'weight': 0, 'input': bitmaps,
                                          'guard' : guard, 'label': guard})
        if u2 in dfa2.final:
            for _, v1, d1 in dfa1.g.edges_iter(u1, data=True):
                if (v1, u2) not in dfa.g:
                    stack.append((v1, u2))
                bitmaps = set(d1['input'])
                guard = d1['guard']
                dfa.g.add_edge((u1, u2), (v1, u2),
                               attr_dict={'weight': 0, 'input': bitmaps,
                                          'guard' : guard, 'label': guard})

    # the set of final states is the product of final sets of dfa1 and dfa2
    dfa.final = set(it.product(dfa1.final, dfa2.final))
    assert len(dfa.final) == 1

    if getDFAType() == DFAType.Infinity:
        mark_product(dfa1, dfa2, dfa, Op.intersection)
    elif getDFAType() == DFAType.Normal:
        # minimize the DFA
        dfa = minimize_dfa(dfa)
    else:
        raise ValueError('DFA type must be either DFAType.Normal or ' +
                         'DFAType.Infinity! {} was given!'.format(getDFAType()))

    # relabel states
    relabel_dfa(dfa)

    logger.debug('[intersection] DFA1: %s DFA2: %s', dfa1.name, dfa2.name)
    return dfa

def union(dfa1, dfa2):
    '''Creates a DFA which accepts the union of the languages corresponding to
    the two DFAs. The disjunction operation of TWTL is mapped to union.
    If an infinity DFA is generated, the corresponding meta-data is copied as
    well.
    '''
    assert dfa1.directed == dfa2.directed and dfa1.multi == dfa2.multi
    assert dfa1.props == dfa2.props
    assert dfa1.alphabet == dfa2.alphabet
    assert len(dfa1.init) == 1 and len(dfa2.init) == 1
    assert len(dfa1.final) == 1 and len(dfa2.final) == 1

    name = '(Union {} {} )'.format(dfa1.name, dfa2.name)
    dfa = Fsa(name, dfa1.props, dfa1.multi)

    # add self-loops on final states and trap states
    attr_dict={'weight': 0, 'input': dfa.alphabet, 'guard' : '(1)',
               'label': '(1)'}
    dfa1.g.add_edges_from([(s, s, attr_dict) for s in dfa1.final])
    dfa2.g.add_edges_from([(s, s, attr_dict) for s in dfa2.final])
    dfa1.add_trap_state()
    dfa2.add_trap_state()
    dfa1.g.remove_edges_from([(s, s) for s in dfa1.final])
    dfa2.g.remove_edges_from([(s, s) for s in dfa2.final])

    init = list(it.product(dfa1.init.keys(), dfa2.init.keys()))
    dfa.init = dict(zip(init, (1,)*len(init)))
    assert len(dfa.init) == 1 # dfa1 and dfa2 are deterministic

    stack = list(init)
    while stack:
        u1, u2 = stack.pop()
        for _, v1, d1 in dfa1.g.edges_iter(u1, data=True):
            for _, v2, d2 in dfa2.g.edges_iter(u2, data=True):
                bitmaps = d1['input'] & d2['input']
                if bitmaps:
                    if (v1, v2) not in dfa.g:
                        stack.append((v1, v2))
                    guard = '({}) & ({})'.format(d1['guard'], d2['guard'])
                    dfa.g.add_edge((u1, u2), (v1, v2),
                                   attr_dict={'weight': 0, 'input': bitmaps,
                                              'guard' : guard, 'label': guard})

    # compute set of final states
    dfa.final = set([(u, v) for u, v in dfa.g.nodes_iter()
                                         if u in dfa1.final or v in dfa2.final])

    # remove trap states
    dfa1.g.remove_nodes_from(['trap'])
    dfa2.g.remove_nodes_from(['trap'])
    dfa.g.remove_nodes_from([('trap', 'trap')])

    # merge finals
    if len(dfa.final) > 1:
        final = (iter(dfa1.final).next(), iter(dfa2.final).next())
        # satisfies both left and right sub-formulae
        choices = dict([(u, Choice(both=d['input']))
                               for u, _, d in dfa.g.in_edges(final, data=True)])
        for u, v, d in dfa.g.in_edges_iter(dfa.final - set([final]), data=True):
            bitmaps = set(d['input'])
            guard = d['guard']
            if dfa.g.has_edge(u, final):
                bitmaps |= dfa.g[u][final]['input']
                guard = '({}) | ({})'.format(guard, dfa.g[u][final]['guard'])
            dfa.g.add_edge(u, final, attr_dict={'weight': 0, 'input': bitmaps,
                                               'guard' : guard, 'label': guard})
            if v[0] in dfa1.final: # satisfies only the left sub-formula
                assert v[1] not in dfa2.final
                choices.setdefault(u, Choice()).left.update(d['input'])
            if v[1] in dfa2.final: # satisfies only the right sub-formula
                assert v[0] not in dfa1.final
                choices.setdefault(u, Choice()).right.update(d['input'])
        # remove all other final states
        dfa.g.remove_nodes_from(dfa.final - set([final]))
        dfa.final = set([final])

    if getDFAType() == DFAType.Infinity:
        mark_product(dfa1, dfa2, dfa, Op.union, choices)
    elif getDFAType() == DFAType.Normal:
        # minimize the DFA
        dfa = minimize_dfa(dfa)
    else:
        raise ValueError('DFA type must be either DFAType.Normal or ' +
                         'DFAType.Infinity! {} was given!'.format(getDFAType()))

    # relabel states
    relabel_dfa(dfa)

    logger.debug('[union] DFA1: %s DFA2: %s', dfa1.name, dfa2.name)
    return dfa

def within(phi, low, high):
    '''Creates either a normal or infinity version DFA corresponding to a within
    operator which encloses the formula corresponding to dfa.
    '''
    assert len(phi.init) == 1 and len(phi.final) == 1

    if getDFAType() == DFAType.Normal:
        return repeat(phi, low, high)
    elif getDFAType() == DFAType.Infinity:
        return eventually(phi, low, high)
    else:
        raise ValueError("Within operator deadline is invalid!")

def eventually(phi_dfa, low, high):
    '''Creates a DFA which accepts the infinity version of a within operator
    which encloses the formula corresponding to phi_dfa.
    NOTE: Assumes that phi_dfa contains no ``trap'' states, i.e. states which do
    not reach a final state.
    '''
    dfa = phi_dfa.clone()
    dfa.name = '(Eventually {} {} {} )'.format(phi_dfa.name, low, high)

    init = dfa.init.keys()[0]
    for state in dfa.g.nodes():
        bitmaps = set()
        guard = '(else)'
        for _, _, d in dfa.g.out_edges_iter(state, data=True):
            bitmaps |= d['input']
        bitmaps = dfa.alphabet - bitmaps

        if state not in dfa.final and bitmaps:
            dfa.g.add_edge(state, init, attr_dict={'weight': 0,
                             'input': bitmaps, 'guard' : guard, 'label': guard})

    # add states to accept a prefix word of any symbol of length low
    if low > 0:
        guard = '(1)'
        bitmaps = dfa.get_guard_bitmap(guard)
        ngen = it.count(start=dfa.g.number_of_nodes())
        nodes = [ngen.next() for _ in range(low)]
        attr_dict={'weight': 0, 'input': bitmaps, 'guard' : guard,
                   'label': guard}
        dfa.g.add_path(nodes, **attr_dict)
        dfa.g.add_edge(nodes[-1], init, attr_dict)
        dfa.init = {nodes[0] : 1}

    # add counter annotation
    mark_eventually(phi_dfa, dfa, low, high)
    logger.debug('[eventually] Low: %d High: %d DFA: %s', low, high,
                 phi_dfa.name)
    return dfa

def repeat(phi_dfa, low, high):
    '''Creates a DFA which accepts the language associated with a within
    operator which encloses the formula corresponding to phi_dfa.
    '''
    assert len(phi_dfa.init) == 1
    assert len(phi_dfa.final) == 1

    init_state = phi_dfa.init.keys()[0]
    final_state = set(phi_dfa.final).pop()

    # remove trap states if there are any
    phi_dfa.remove_trap_states()
    # initialize the resulting dfa
    name = '(Repeat {} {} {} )'.format(phi_dfa.name, low, high)
    dfa = Fsa(name, phi_dfa.props, phi_dfa.multi)
    # compute the maximum number of restarts
    b = nx.shortest_path_length(phi_dfa.g, source=init_state,
                                target=final_state)
    d = high - low - b + 2
    # copy dfa to dfa_aux and initialize the list of restart states
    inits = []
    nstates = 0
    for k in range(d):
        # 1. relabel dfa_aux
        mapping = dict(zip(phi_dfa.g.nodes_iter(),
                         range(nstates, nstates + phi_dfa.g.number_of_nodes())))
        mapping[final_state] = -1 # mark final state as special
        dfa_aux = relabel_dfa(phi_dfa, mapping, copy=True)
        # 2. compute truncated dfa_aux
        truncate_dfa(dfa_aux, cutoff=(high-low+1)-k)
        # 3. add truncated dfa_aux to dfa
        dfa.g.add_edges_from(dfa_aux.g.edges_iter(data=True))
        inits.append(dfa_aux.init.keys()[0])
        nstates += dfa_aux.g.number_of_nodes()
    # set initial and final state
    dfa.init = {inits[0] : 1}
    dfa.final = set([-1])
    # create restart transitions
    current_states = set([inits[0]])
    for rstate in inits[1:]: # current restart state
        next_states = set([])
        # connect current states to restart state for (else) symbols
        for state in current_states:
            bitmaps = set()
            guard = '(else)'
            for _, next_state, d in dfa.g.out_edges_iter(state, data=True):
                bitmaps |= d['input']
                next_states.add(next_state)
            bitmaps = dfa.alphabet - bitmaps

            if state not in dfa.final and bitmaps:
                dfa.g.add_edge(state, rstate,
                               attr_dict={'weight': 0, 'input': bitmaps,
                                          'guard' : guard, 'label': guard})
        # update current states
        current_states = next_states | set([rstate])

    # relabel states
    relabel_dfa(dfa)
    # add states to accept a prefix word of any symbol of length low
    if low > 0:
        guard = '(1)'
        bitmaps = dfa.get_guard_bitmap(guard)
        ngen = it.count(start=dfa.g.number_of_nodes())
        nodes = [ngen.next() for _ in range(low)]
        attr_dict={'weight': 0, 'input': bitmaps, 'guard' : guard,
                   'label': guard}
        dfa.g.add_path(nodes, **attr_dict)
        dfa.g.add_edge(nodes[-1], dfa.init.keys()[0], attr_dict)
        dfa.init = {nodes[0] : 1}

    logger.debug('[within] Low: %d High: %d DFA: %s', low, high, phi_dfa.name)
    return dfa

def truncate_dfa(dfa, cutoff):
    '''Returns a dfa which accepts only the words of length at most cutoff from
    the language associated with the given dfa.
    Note: It assumes that the given dfa has a finite language, i.e. it is a DAG.

    Adapted from networkx.algorithms.shortest_paths.unweighted.\
    single_source_shortest_path_length
    NetworkX is available at http://networkx.github.io.
    '''
    assert len(dfa.init) == 1 # deterministic model
    source = dfa.init.keys()[0]

    # compute transitions which form path of length at most cutoff in the dfa
    visited = set([source])
    edges = []
    nextlevel = [(source, iter(dfa.g[source]))]
    for _ in range(cutoff):
        thislevel = nextlevel
        nextlevel = []
        for parent, children in thislevel:
            for child in children:
                edges.append((parent, child))
                if child not in visited:
                    visited.add(child)
                    nextlevel.append((child, iter(dfa.g[child])))

    # remove transitions which are not part of paths of length at most cutoff
    dfa.g.remove_edges_from([e for e in dfa.g.edges_iter() if e not in edges])
    # remove all states which do not reach the final states
    dfa.remove_trap_states()

    return dfa
