license_text='''
    Module contains various tests of TWTL translation and operations.
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
.. module:: tests.py
   :synopsis: Module contains various tests.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

import matplotlib.pyplot as plt

from dfa import accept_prop, complement, concatenation, eventually, hold,\
                intersection, repeat, union, within
from dfa import DFAType, setDFAType


def test_accept_prop_boolean(verbose=False, show=False):
    props = ['A', 'B', 'C']

    for kwargs in [{'boolean':True}, {'boolean':False}, {'prop':'A'}]:
        dfa = accept_prop(props, **kwargs)
        if verbose:
            print dfa
        if show:
            dfa.visualize(draw='matplotlib')
            plt.show()

def test_complement(verbose=False, show=False):
    pass

def test_intersection_union(verbose=False, show=False):
    props = ['A', 'B', 'C']

    dfa1 = hold(props, prop='A', duration=1, negation=False)
    dfa1 = within(dfa1, low=1, high=5)

    dfa2 = hold(props, prop='B', duration=2, negation=False)
    dfa2 = within(dfa2, low=0, high=4)

    for operation in [union]: #intersection, union]:
        if verbose:
            print 'Operation:', operation.__name__

        dfa = operation(dfa1, dfa2)
        if verbose:
            for fsa in [dfa1, dfa2, dfa]:
                print fsa
                print fsa.tree
                print fsa.counters
                print
                for u, v, d in fsa.g.edges_iter(data=True):
                    print (u, v), d
        if show:
            dfa1.visualize(draw='matplotlib')
            plt.show()
            dfa2.visualize(draw='matplotlib')
            plt.show()
            dfa.visualize(draw='matplotlib')
            plt.show()

def test_concatenation(verbose=False, show=False):
    props = ['A', 'B', 'C']

    dfa1 = hold(props, prop='A', duration=2, negation=False)
    dfa1 = eventually(dfa1, low=1, high=5)

    dfa2 = hold(props, prop='B', duration=3, negation=False)
    dfa2 = eventually(dfa2, low=0, high=4)

    dfa = concatenation(dfa1, dfa2)
    if verbose:
        print dfa
        print dfa.tree
        print dfa.counters
        print
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa1.visualize(draw='matplotlib')
        plt.show()
        dfa2.visualize(draw='matplotlib')
        plt.show()
        dfa.visualize(draw='matplotlib')
        plt.show()

def test_hold(verbose=False, show=False):
    props = ['A', 'B', 'C']

    for kwargs in [{'prop':'A', 'duration':2, 'negation':False},
                   {'prop':'B', 'duration':2, 'negation':True}]:
        dfa = hold(props, **kwargs)
        if verbose:
            print dfa
        if show:
            dfa.visualize(draw='matplotlib')
            plt.show()

def test_repeat(verbose=False, show=False):
    setDFAType(DFAType.Normal)

    props = ['A', 'B', 'C']

    dfa = hold(props, prop='A', duration=2, negation=False)
    dfa = repeat(dfa, low=2, high=4)
    if verbose:
        print dfa
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize()
        plt.show()

    # Test with low zero
    dfa = hold(props, prop='A', duration=2, negation=False)
    dfa = repeat(dfa, low=0, high=4)
    if verbose:
        print dfa
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize()
        plt.show()

    # Test with trap state
    dfa = hold(props, prop='A', duration=2, negation=False)
    dfa.add_trap_state()
    dfa = repeat(dfa, low=0, high=4)
    if verbose:
        print dfa
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize()
        plt.show()

    # Test with truncated dfa
    dfa1 = hold(props, prop='A', duration=2, negation=False)
    dfa2 = hold(props, prop='B', duration=3, negation=False)
    dfa = union(dfa1, dfa2)

    if verbose:
        print '[test_within] Preprocessing:'
        print dfa1
        print dfa2
        print 'Union'
        print dfa
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d

    dfa = repeat(dfa, low=0, high=4)
    if verbose:
        print dfa
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize()
        plt.show()

def test_eventually(verbose=False, show=False):
    setDFAType(DFAType.Infinity)

    props = ['A', 'B', 'C']

    dfa = hold(props, prop='A', duration=2, negation=False)
    dfa = eventually(dfa, low=2, high=5)
    if verbose:
        print dfa
        print dfa.tree
        print dfa.counters
        print
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize(draw='matplotlib')
        plt.show()

    # Test with low zero
    dfa = hold(props, prop='A', duration=2, negation=False)
    dfa = eventually(dfa, low=0, high=5)
    if verbose:
        print dfa
        print dfa.tree
        print dfa.counters
        print
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize(draw='matplotlib')
        plt.show()

    # Test with trap state
    dfa = hold(props, prop='A', duration=2, negation=False)
    dfa.add_trap_state()
    dfa = eventually(dfa, low=0, high=5)
    if verbose:
        print dfa
        print dfa.tree
        print dfa.counters
        print
        for u, v, d in dfa.g.edges_iter(data=True):
            print (u, v), d
    if show:
        dfa.visualize(draw='matplotlib')
        plt.show()

################################################################################

if __name__ == '__main__':

    verbose = True
    visualize = True

    # test 1: accept prop/boolean
#     test_accept_prop_boolean(verbose=verbose, show=visualize)
#     # test 2: complement
#     test_complement(verbose=verbose, show=visualize)
    # test 3: intersection/union
    #setDFAType(DFAType.Infinity)
    test_intersection_union(verbose=verbose, show=visualize)
    # test 5: concatenation
#     test_concatenation(verbose=verbose, show=visualize)
    # test 6: hold
#     test_hold(verbose=verbose, show=visualize)
    # test 7: repeat
#     test_repeat(verbose=verbose, show=visualize)
    # test 8: eventually
#     test_eventually(verbose=verbose, show=visualize)
