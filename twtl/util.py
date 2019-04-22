license_text='''
    Module implements utility functions used throughout the code.
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
.. module:: util.py
   :synopsis: Module implements utility functions used in throughout the code.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

import logging
from dfa import Op
from StringIO import StringIO

def _debug_pprint_tree(tree, level=0, indent=2):
    '''Returns a multi-line string representation of the whole tree.'''
    ret = StringIO()
    print>>ret, ' '*(level*indent), str(tree)
    print>>ret, ' '*(level*indent), 'Init:', tree.init
    print>>ret, ' '*(level*indent), 'Final:', tree.final
    if tree.operation == Op.event:
        print>>ret, ' '*(level*indent), 'High:', tree.high, 'Low:', tree.low
        print>>ret, ' '*(level*indent), 'Active:', tree.active, 'Tau:', tree.tau
    if tree.operation == Op.union:
        print>>ret, ' '*(level*indent), 'Choices:'
        for k, v in tree.choices.iteritems():
            print>>ret, ' '*((level+1)*indent), k, '->', v
    if tree.left is not None:
        print>>ret, ' '*(level*indent), 'Left:'
        print>>ret, _debug_pprint_tree(tree.left, level=level+1),
    if tree.right is not None:
        print>>ret, ' '*(level*indent), 'Right:'
        print>>ret, _debug_pprint_tree(tree.right, level=level+1),
    ret_str = str(ret.getvalue())
    ret.close()
    return ret_str

def _debug_control_paths(op, M_left, M_right, M):
    '''Tests whether the given containers are indeed sets.'''
    if logging.getLogger().isEnabledFor(logging.DEBUG):
        assert len([p.path for p in M_left.paths]) == len(set([p.path for p in M_left.paths]))
        assert len([p.path for p in M_right.paths]) == len(set([p.path for p in M_right.paths]))
        assert len([p.path for p in M.paths]) == len(set([p.path for p in M.paths]))
        logging.debug('%s:', op)
        logging.debug('Left:\n%s', M_left.paths)
        logging.debug('Right:\n%s', M_right.paths)
        logging.debug('%s:\n%s', op, M.paths)

def _debug_control_paths_endpoints_check(tree, M):
    '''Tests whether the end state of a (partial) control path is a final state
    for the operation associated with the root of the given tree.
    '''
    if logging.getLogger().isEnabledFor(logging.DEBUG):
            assert all([p.path[-1][1] in tree.final for p in M])

def _debug_eventually_info(tree, paths):
    '''Prints out information about paths corresponding to an eventually
    operator.
    '''
    logging.debug('[WWF] tree: init: %s, final: %s', tree.init, tree.final)
    if logging.getLogger().isEnabledFor(logging.DEBUG):
        logging.debug('Data: %s', str(tree))
        logging.debug('Data init: %s', tree.init)
        logging.debug('Data finish: %s', tree.final)
        logging.debug('Paths: %s', paths)
        logging.debug('Tau:%s', [len(path) - tree.high - 1 for path in paths])
        assert all([p[-1][1] in tree.final for p in paths])

def _debug_print_disj_choices(tree, constraint, c_left=None, c_right=None):
    '''Prints the choices and constraints associated with a disjunction
    operator. It also prints the updated constrains for the left and right
    sub-trees if these are provided, respectively.
    '''
    if constraint is not None:
        for k, v in constraint.iteritems():
            logging.debug('constraint: %s -> %s', k, v)
    for k, v in tree.choices.iteritems():
        logging.debug('tree.choices: %s -> %s', k, v)

    if c_left is not None:
        for k, v in c_left.iteritems():
            logging.debug('c_left: %s -> %s', k, v)
    if c_right is not None:
        for k, v in c_right.iteritems():
            logging.debug('c_right: %s -> %s', k, v)

def _debug_no_repeated_finals_check(pa, tree, M):
    '''Tests whether all paths in M intersect the set of finals states of pa
    exactly once.
    '''
    if logging.getLogger().isEnabledFor(logging.DEBUG):
        finish = set([p for p in pa.g.nodes_iter() if p[1] in tree.final])
        assert all([p.path[-1] in finish for p in M])
        for p in M:
            print 'assert:', finish, p
            assert not (finish & set(p.path[:-1]))
        assert all([p.path[-1][1] in tree.final for p in M])

def _debug_not_wwf_eventually_check(tree, sat_paths, path):
    '''Checks whether the lengths of the given paths, after adding the prefix
    delay is consistent.
    '''
    if logging.getLogger().isEnabledFor(logging.DEBUG):
        assert all([len(p.path)==(tree.low+len(path)) for p in sat_paths])


if __name__ == '__main__':
    pass
