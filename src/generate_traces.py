license_text='''
    Module generates random traces for the learning procedure.
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
.. module:: generate_traces.py
   :synopsis: Module generates traces for the learning procedure.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''

from pprint import pprint

import numpy as np

from twtl import translate, DFAType

def generate(formula, traces_file, no_traces=100):
    np.random.seed(1001)

    alphabet, dfa = translate(formula, kind=DFAType.Infinity, optimize=True)
    alphabet = [set([s]) for s in alphabet] + [set()]

    print 'alphabet:', alphabet

    data = {'positive': [], 'negative': []}

    both = 0

    for k in range(no_traces):
        print 'k:', k

        state = iter(dfa.init).next()
        w = []
        while True:
            r = np.random.randint(0, len(alphabet))
            n = int(np.ceil(1 * np.random.exponential(3)))
#             print r, n
            symbol = alphabet[r]
            for _ in range(n):
                w.append(symbol)
                states = dfa.next_states_of_fsa(state, symbol)
                assert len(states) <= 1
                if not states:
                    break
                state = states[0]
            if state in dfa.final:
                ch = np.random.randint(0, 2)
                if len(w) < 62:
                    data['positive'].append(w)
                elif len(w) > 82:
                    data['negative'].append(w)
                elif ch:
                    both += 1
                    data['positive'].append(w)
                else:
                    both += 1
                    data['negative'].append(w)
                break

    print 'done', len(data['positive']), len(data['negative']), both

#     for w in data['positive']:
#         print 'length:', len(w)
#     print
#     for w in data['negative']:
#         print 'length:', len(w)

    with open(traces_file, 'w') as fout:
        pprint(data, fout, width=5000)

if __name__ == '__main__':
    generate('[H^2 A]^[0, 4] * [H^3 B]^[2, 6] * [H^2 C]^[0, 3]',
             '../data/traces.txt')
