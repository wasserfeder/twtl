license_text='''
    Case studies for Theoretical Computer Science journal.
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
.. module:: examples_tcs.py
   :synopsis: Case studies for Theoretical Computer Science journal.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

import logging, sys
import StringIO

import networkx as nx
import matplotlib.pyplot as plt

import twtl
from dfa import DFAType
from synthesis import expand_duration_ts, compute_control_policy, ts_times_fsa,\
                      verify
from learning import learn_deadlines
from lomap import Ts


def case1_synthesis(formula, ts_file):
    _, dfa_0, dfa_inf, bdd = twtl.translate(formula, kind='both', norm=True)

    logging.debug('alphabet: {}'.format(dfa_inf.props))

    for u, v, d in dfa_inf.g.edges_iter(data=True):
        logging.debug('({}, {}): {}'.format(u, v, d))

    dfa_inf.visualize(draw='matplotlib')
    plt.show()

    logging.debug('\nEnd of translate\n\n')

    logging.info('The bound of formula "%s" is (%d, %d)!', formula, *bdd)
    logging.info('Translated formula "%s" to normal DFA of size (%d, %d)!',
                 formula, *dfa_0.size())
    logging.info('Translated formula "%s" to infinity DFA of size (%d, %d)!',
                 formula, *dfa_inf.size())

    logging.debug('\n\nStart policy computation\n')

    ts = Ts.load(ts_file)
    ets = expand_duration_ts(ts)

    for name, dfa in [('normal', dfa_0), ('infinity', dfa_inf)]:
        logging.info('Constructing product automaton with %s DFA!', name)
        pa = ts_times_fsa(ets, dfa)
        logging.info('Product automaton size is: (%d, %d)', *pa.size())

        if name == 'infinity':
            for u in pa.g.nodes_iter():
                logging.debug('{} -> {}'.format(u, pa.g.neighbors(u)))

            pa.visualize(draw='matplotlib')
            plt.show()

        # compute optimal path in PA and then project onto the TS
        policy, output, tau = compute_control_policy(pa, dfa, dfa.kind)
        logging.info('Max deadline: %s', tau)
        if policy is not None:
            logging.info('Generated output word is: %s', [tuple(o) for o in output])

            policy = [x for x in policy if x not in ets.state_map]
            out = StringIO.StringIO()
            for u, v in zip(policy[:-1], policy[1:]):
                print>>out, u, '->', ts.g[u][v][0]['duration'], '->',
            print>>out, policy[-1],
            logging.info('Generated control policy is: %s', out.getvalue())
            out.close()

            logging.info('Relaxation is: %s',
                         twtl.temporal_relaxation(output, formula=formula))
        else:
            logging.info('No control policy found!')

def case2_verification(formula, ts_file):
    _, dfa_inf, bdd  = twtl.translate(formula, kind=DFAType.Infinity, norm=True)

    logging.info('The bound of formula "%s" is (%d, %d)!', formula, *bdd)
    logging.info('Translated formula "%s" to infinity DFA of size (%d, %d)!',
                 formula, *dfa_inf.size())

    ts = Ts.load(ts_file)
    ts.g.add_edges_from(ts.g.edges(), weight=1)

    for u, v in ts.g.edges_iter():
        print u, '->', v

    result = verify(ts, dfa_inf)
    logging.info('The result of the verification procedure is %s!', result)

def case3_learning(formula, traces_file):
    with open(traces_file, 'r') as fin:
        data = eval(''.join(fin.readlines()))
    traces_p, traces_n = data['positive'], data['negative']
    deadlines, mcr = learn_deadlines(formula, traces_p, traces_n)
    logging.info('The inferred deadlines are: %s with misclassification rate: %s',
                 deadlines, mcr)

def setup_logging():
    fs, dfs = '%(asctime)s %(levelname)s %(message)s', '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logging.basicConfig(filename='../data/examples_tcs.log', level=loglevel,
                        format=fs, datefmt=dfs)

    root = logging.getLogger()
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(loglevel)
    ch.setFormatter(logging.Formatter(fs, dfs))
    root.addHandler(ch)

if __name__ == '__main__':
    setup_logging()
    # case study 1: Synthesis
    phi = '[H^2 A]^[0, 6] * ([H^1 B]^[0, 3] | [H^1 C]^[1, 4]) * [H^1 D]^[0, 6]'
    case1_synthesis(phi, '../data/ts_synthesis.yaml')
    # case study 2: Verification
    phi1 = '[H^1 A]^[1, 2]'
    case2_verification(phi1, '../data/ts_verification.yaml')
    phi2 = '[H^3 !B]^[1, 4]'
    case2_verification(phi2, '../data/ts_verification.yaml')
    # case study 3: Learning
    phi_learn = '[H^1 A]^[0, 2] * [H^2 B]^[0, 3]'
    case3_learning(phi_learn, '../data/traces_simple.txt')
    phi_learn = '[H^2 A]^[0, 4] * [H^3 B]^[2, 6] * [H^2 C]^[0, 3]'
    case3_learning(phi_learn, '../data/traces.txt')
