license_text='''
    Module implements API for translating a TWTL formula to a DFA.
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
.. module:: twtl.py
   :synopsis: Module implements API for translating a TWTL formula to a DFA.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

import logging
import itertools as it

from antlr4 import InputStream, CommonTokenStream

from twtlLexer import twtlLexer
from twtlParser import twtlParser
from twtl_ast import TWTLAbstractSyntaxTreeExtractor
from twtl2dfa import twtl2dfa
from dfa import setDFAType, DFAType, Op, setOptimizationFlag
from util import _debug_pprint_tree


def monitor(formula=None, kind=None, dfa=None, cutoff=None):
    '''Creates a monitor for a TWTL formula.
    It accept the following combinations of parameters:
    1) a formula and its kind;
    2) a dfa, in which case kind is silently ignored.
    In either case a dfa is available for monitoring.
    The cutoff parameter limits the maximum range a trajectory is monitored.
    If it is absent, then the cutoff horizon is the formula upper norm or
    infinite for the normal and infinity versions of the TWTL formula,
    respectively.
    '''
    # either compute infinity automaton or use the one provided
    if formula is None and dfa is None:
        raise Exception('Must provide either a TWTL formula or an automaton!')
    elif dfa is None:
        _, dfa = translate(formula, kind=kind)
    kind = dfa.kind

    if  kind == DFAType.Normal:
        if formula is not None:
            seq = range((norm(formula) if cutoff is None else cutoff) + 1)
        else:
            seq = range(cutoff + 1)
    elif kind == DFAType.Infinity:
        seq = it.count() if cutoff is None else xrange(cutoff + 1)
    else:
        raise ValueError('DFA type must be either DFAType.Normal, ' +
                       'DFAType.Infinity or "both"! {} was given!'.format(kind))

    state = dfa.init.keys()[0]
    ret = 0
    for _ in seq:
        symbol = yield ret
        r = dfa.next_states_of_fsa(state, symbol)
        assert len(r) <= 1, 'Should be deterministic!'
        if r:
            state = r[0]
            ret = 1*(state in dfa.final)
        else:
            break
    while True:
        yield -1

def _init_tree(tree):
    '''Initialized the tree for computing the temporal relaxations.'''
    stack = [tree]
    while stack:
        t = stack.pop()
        if t.operation == Op.event:
            t.active = False
            t.done = False
            t.tau = -1
        if t.left is not None:
            stack.append(t.left)
        if t.right is not None:
            stack.append(t.right)

def _update_tree(tree, state, prev_state, symbol, constraint=None):
    '''Updated the activity and tau values of all eventually operators based on
    the current state, the previous state and the previous symbol. The
    ``constraints'' parameter is used to choose which part of the formula is
    considered when evaluating disjunction operators.
    '''
    if tree.unr:
        return
    if tree.operation == Op.event:
        if state in tree.init:
            tree.active = True
        if state in tree.final:
            if constraint is None:
                tree.active = False
                tree.done = True
            elif set([symbol]) <= constraint.get(prev_state, set()):
                tree.active = False
                tree.done = True
        if tree.active:
            tree.tau += 1
        if not tree.wwf:
            _update_tree(tree.left, state, prev_state, symbol, constraint)
    elif tree.operation == Op.cat:
        _update_tree(tree.left, state, prev_state, symbol)
        _update_tree(tree.right, state, prev_state, symbol, constraint)
    elif tree.operation == Op.intersection:
        _update_tree(tree.left, state, prev_state, symbol, constraint)
        _update_tree(tree.right, state, prev_state, symbol, constraint)
    elif tree.operation == Op.union:
        if constraint is None:
            c_left = {s: ch.both | ch.left for s, ch in tree.choices.iteritems()}
            c_right = {s: ch.both | ch.right for s, ch in tree.choices.iteritems()}
        else:
            c_left = dict()
            c_right = dict()
            for s in tree.choices.viewkeys() & constraint.viewkeys():
                c_left[s] = constraint[s] & (tree.choices[s].both | tree.choices[s].left)
                c_right[s] = constraint[s] & (tree.choices[s].both | tree.choices[s].right)
        _update_tree(tree.left, state, prev_state, symbol, c_left)
        _update_tree(tree.right, state, prev_state, symbol, c_right)

def _eval_relaxation(tree):
    '''Evaluates the tau values and returns the maximum deadline and the
    associated valuation of all the tau values.
    '''
    if tree.unr:
        return float('-Inf'), []
    if tree.wwf and tree.operation == Op.event:
        if not tree.done:
            tree.tau = float('-Inf')
        else:
            tree.tau -= tree.high
        return tree.tau, [(tree.tau, (tree.low, tree.high))]
    if not tree.wwf and tree.operation == Op.event:
        t_opt_left, tau_left = _eval_relaxation(tree.left)
        if not tree.done:
            tree.tau = float('-Inf')
            return tree.tau, tau_left + [(tree.tau, (tree.low, tree.high))]
        else:
            tree.tau -= tree.high
            return max(tree.tau, t_opt_left), tau_left + [(tree.tau, (tree.low, tree.high))]
    if tree.operation in (Op.cat, Op.intersection, Op.union):
        t_opt_left, tau_left = _eval_relaxation(tree.left)
        t_opt_right, tau_right = _eval_relaxation(tree.right)
        if tree.operation in (Op.cat, Op.intersection):
            if t_opt_left > float('-Inf') and t_opt_left > float('-Inf'):
                return max(t_opt_left, t_opt_right), tau_left + tau_right
            else:
                return float('-Inf'), tau_left + tau_right
        else:
            if t_opt_left > float('-Inf') and t_opt_left > float('-Inf'):
                return min(t_opt_left, t_opt_right), tau_left + tau_right
            else:
                return max(t_opt_left, t_opt_right), tau_left + tau_right

def temporal_relaxation(word, formula=None, dfa=None):
    '''Computes the temporal relaxation of the given formula or dfa such that
    the given word satisfies the formula or is accepted by the automaton,
    respectively.
    Note: If an automaton is specified, it must not be optimized.
    '''
    # either compute infinity automaton or use the one provided
    if formula is None and dfa is None:
            raise Exception('Must provide either a TWTL formula or'
                            + ' an infinity automaton!')
    elif dfa is None:
        _, dfa = translate(formula, kind=DFAType.Infinity, optimize=False)
    assert dfa.kind == DFAType.Infinity

    # initialize tree
    _init_tree(dfa.tree)
#     logging.debug('Init:\n%s', _debug_pprint_tree(dfa.tree))

    prev_state = None
    state = dfa.init.keys()[0]
    prev_w = set()
    for w in word + [set([])]: # hack to catch the last state
        # start/stop counters and increment all active counters
        _update_tree(dfa.tree, state, prev_state, dfa.bitmap_of_props(prev_w))
#         logging.debug('Update: state=%s prev_state=%s w=%s prev_w=%s final=%s',
#                       state, prev_state, w, prev_w, dfa.final)
#         logging.debug('Update:\n%s', _debug_pprint_tree(dfa.tree))
        # test for satisfaction
        if state in dfa.final:
            break
        else: # compute next state
            r = dfa.next_states(state, w)
            assert len(r) == 1, 'Should be deterministic!'
            prev_state = state
            state = r[0]
        prev_w = w
    # substract deadlines from counter values to obtain the tau values
    return _eval_relaxation(dfa.tree)

def norm(formula):
    '''Computes the bounds of the given TWTL formula and returns a 2-tuple
    containing the lower and upper bounds, respectively.
    '''
    lexer = twtlLexer(InputStream(formula))
    tokens = CommonTokenStream(lexer)
    parser = twtlParser(tokens)
    phi = parser.formula()

    # AST
    ast = TWTLAbstractSyntaxTreeExtractor().visit(t)

    # compute TWTL bound
    return ast.bounds()

def translate(formula, kind='both', norm=False, optimize=True):
    '''Converts a TWTL formula into an FSA. It can returns both a normal FSA or
    the automaton corresponding to the relaxed infinity version of the
    specification.
    If kind is: (a) DFAType.Normal it returns only the normal version;
    (b) DFAType.Infinity it returns only the relaxed version; and
    (c) 'both' it returns both automata versions.
    If norm is True then the bounds of the TWTL formula are computed as well.

    The functions returns a tuple containing in order: (a) the alphabet;
    (b) the normal automaton (if requested); (c) the infinity version automaton
    (if requested); and (d) the bounds of the TWTL formula (if requested).

    The ``optimize'' flag is used to specify that the annotation data should be
    optimized. Note that the synthesis algorithm assumes an optimized automaton,
    while computing temporal relaxations is performed using an unoptimized
    automaton.
    '''
    if kind == 'both':
        kind = [DFAType.Normal, DFAType.Infinity]
    elif kind in [DFAType.Normal, DFAType.Infinity]:
        kind = [kind]
    else:
        raise ValueError('DFA type must be either DFAType.Normal, ' +
                         'DFAType.Infinity or "both"! {} was given!'.format(kind))

    lexer = twtlLexer(InputStream(formula))
    tokens = CommonTokenStream(lexer)
    parser = twtlParser(tokens)
    phi = parser.formula()

    # AST
    ast = TWTLAbstractSyntaxTreeExtractor().visit(t)

    alphabet = ast.propositions()
    result= [alphabet]

    if DFAType.Normal in kind:
        setDFAType(DFAType.Normal)
        dfa = twtl2dfa(ast, alphabet)
        dfa.kind = DFAType.Normal
        result.append(dfa)

    if DFAType.Infinity in kind:
        setDFAType(DFAType.Infinity)
        setOptimizationFlag(optimize)
        dfa_inf = twtl2dfa(ast, alphabet)
        dfa_inf.kind = DFAType.Infinity
        result.append(dfa_inf)

    if norm: # compute TWTL bound
        result.append(ast.bounds())

    if logging.getLogger().isEnabledFor(logging.DEBUG):
        for mode, name in [(DFAType.Normal, 'Normal'),
                           (DFAType.Infinity, 'Infinity')]:
            if mode not in kind:
                continue
            elif mode == DFAType.Normal:
                pdfa = dfa
            else:
                pdfa = dfa_inf
            logging.debug('[spec] spec: {}'.format(formula))
            logging.debug('[spec] mode: {} DFA: {}'.format(name, pdfa))
            if mode == DFAType.Infinity:
                logging.debug('[spec] tree:\n{}'.format(pdfa.tree.pprint()))
            logging.debug('[spec] No of nodes: {}'.format(pdfa.g.number_of_nodes()))
            logging.debug('[spec] No of edges: {}'.format(pdfa.g.number_of_edges()))

    return tuple(result)

if __name__ == '__main__':
#     print translate('[H^3 !A]^[0, 8] * [H^2 B & [H^4 C]^[3, 9]]^[2, 19]',
#                     kind=DFAType.Normal, norm=True)

    res = translate('[H^2 A]^[0, 4] | [H^2 B]^[2, 5]',
                    kind=DFAType.Infinity, norm=True)

    print res
    print res[1].g.nodes()
