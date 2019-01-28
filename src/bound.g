/**
 *  Tree grammar used to compute the bound of a TWTL formula.
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

tree grammar bound;

options {
    language = Python;
    tokenVocab=twtl;
    ASTLabelType=CommonTree;
}

@header {
license_text='''
    Module computes the bound of a TWTL formula.
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
}

@members {
    bound = 0

    def getBound(self):
        return self.bound
}

eval:   formula {self.bound = $formula.value;}
    ;

formula returns [value]
    :   ^(OR  a=formula b=formula) {$value = (min(a[0], b[0]), max(a[1], b[1]))}
    |   ^(AND a=formula b=formula) {$value = (max(a[0], b[0]), max(a[1], b[1]))}
    |   ^(NOT a=formula)           {$value = a}
    |   ^(CONCAT a=formula b=formula) {$value = [x+y+1 for x, y in zip(a, b)]}
    |   ^(HOLD INT PROP)        {$value = (int($INT.text), int($INT.text))}
    |   ^(HOLD INT ^(NOT PROP)) {$value = (int($INT.text), int($INT.text))}
    |   ^(WITHIN phi=formula low=INT high=INT) {
            $value = (int($low.text)+phi[0], int($high.text)) # - int($low.text)
            if phi[1] > int($high.text) - int($low.text):
                raise ValueError("Within operator deadline is invalid!")
        }
    |   PROP                        {$value = (0, 0);}
    |   TRUE                        {$value = (0, 0);}
    |   FALSE                       {$value = (0, 0);}
    ;
