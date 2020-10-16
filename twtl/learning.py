license_text='''
    Module implements automata-based learning algorithms with TWTL properties.
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
.. module:: learning.py
   :synopsis: Module implements automata-based learning algorithms with TWTL
              properties.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''
import itertools as it
import logging

import numpy as np

from twtl import DFAType, translate, temporal_relaxation

def get_deadlines(word, dfa):
    _, data = temporal_relaxation(word, dfa=dfa)
    return [tau + high for tau, (_, high) in data]

def learn_deadlines(formula, traces_p, traces_n):
    '''Infers deadlines for a given TWTL formula from the lists of positive and
    negative traces. The procedure generates the annotated DFA corresponding to
    the given formula just once and uses it to compute tight deadlines for all
    traces, positive and negative. Lastly, the deadlines are aggregated using a
    simple heuristic that ignores interdependencies between deadlines.
    '''
    # 1. construct annotated DFA
    _, dfa = translate(formula, kind=DFAType.Infinity, optimize=True)

    # 2. compute tightest deadlines for all traces
    D_p = [get_deadlines(word, dfa) for word in traces_p]
    D_n = [get_deadlines(word, dfa) for word in traces_n]

    logging.debug('Tight deadlines for positive traces: %s', D_p)
    logging.debug('Tight deadlines for negative traces: %s', D_n)

    m = len(D_p[0])
    d = [-float('Inf')]*m
    for k in range(m):
        deadlines = sorted(set([b[k] for b in it.chain(D_p, D_n)]))
        min_dl = None
        mcr = float('Inf')
        for dl in deadlines:
            FP = [dl_n[k] for dl_n in D_n if dl_n[k] <= dl]
            FN = [dl_p[k] for dl_p in D_p if dl_p[k] >  dl]
            logging.debug('Deadline id: %d, value %d, MCR: %d, FP: %s, FN: %s',
                          k, dl, len(FP) + len(FN), FP, FN)
            if len(FP) + len(FN) < mcr:
                mcr = len(FP) + len(FN)
                min_dl = dl
            logging.debug('Tightest deadline: %d, MCR: %d', mcr, min_dl)
        d[k] = min_dl

    # compute actual MCR - FIXME: this won't work with disjunction
    d_vec = np.asarray(d)
    MCR = np.sum([np.any(np.asarray(d_p) > d_vec) for d_p in D_p]) \
          + np.sum([np.all(np.asarray(d_n) <= d_vec) for d_n in D_n])

    return d, MCR
