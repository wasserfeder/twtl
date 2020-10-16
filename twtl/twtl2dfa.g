/**
 *  Tree grammar used to translate a TWTL formula to an automaton.
 *  Copyright (C) 2015-2016  Cristian Ioan Vasile <cvasile@bu.edu>
 *  Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab,
 *  Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

tree grammar twtl2dfa;

options {
    language = Python;
    tokenVocab=twtl;
    ASTLabelType=CommonTree;
}

@header {
license_text='''
    Module converts a TWTL formula to an automaton.
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

from dfa import accept_prop, complement, concatenation, hold, intersection, \
                union, within
}

@members {
    dfa=None
    props = None

    def getDFA(self):
        return self.dfa
}

eval:   formula {self.dfa = $formula.dfa;}
    ;

formula returns [dfa]
    :   ^(OR  a=formula b=formula)  {$dfa = union(a, b)}
    |   ^(AND a=formula b=formula)  {$dfa = intersection(a, b)}
    |   ^(NOT a=formula)            {$dfa = complement(a)}
    |   ^(CONCAT a=formula b=formula)  {$dfa = concatenation(a, b)}
    |   ^(HOLD INT p=PROP) {
            $dfa = hold(self.props, p.text, int($INT.text), negation=False)
        }
    |   ^(HOLD INT ^(NOT p=PROP)) {
            $dfa = hold(self.props, p.text, int($INT.text), negation=True)
        }
    |   ^(WITHIN phi=formula low=INT high=INT) {
            $dfa = within(phi, int($low.text), int($high.text))
        }
    |   PROP                        {$dfa = accept_prop(self.props, prop=$PROP)}
    |   TRUE                        {$dfa = accept_prop(self.props, boolean=True)}
    |   FALSE                       {$dfa = accept_prop(self.props, boolean=False)}
    ;
