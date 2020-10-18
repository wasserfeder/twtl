/**
 *  Grammar file for Time Window Temporal Logic.
 *  Copyright (C) 2015-2020  Cristian Ioan Vasile <cvasile@lehigh.edu>
 *  Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics Lab
 *  Lehigh University
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
grammar twtl;

@header {
license_text='''
    Copyright (C) 2015-2020  Cristian Ioan Vasile <cvasile@lehigh.edu>
    Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics Lab
    Lehigh University

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


//parser entry point
formula:
      op='(' child=formula ')'
    | left=formula op=OR right=formula
    | left=formula op=AND right=formula
    | (op=HOLD '^' duration=INT)? (negated=NOT)? prop=(TRUE | FALSE | PROP)
    | op=NOT child=formula
    | left=formula op=CONCAT right=formula
    | op='[' child=formula ']' '^' '[' low=INT ',' high=INT ']'
    ;

//fragments
fragment
DIGIT       : ('0'..'9'); //digit
fragment
LWLETTER    : ('a'..'z'); //lower case letter
fragment
HGLETTER    : ('A'..'Z'); //upper case letter
fragment
//allowed upper case letter except H and W
HGLETTERALL : ('A'..'G') | ('I' .. 'V') | ('X' .. 'Z');
fragment
LETTER      : LWLETTER | HGLETTER; //letter

//tokens
// Boolean operators
AND : '&' | '&&';
OR  : '|' | '||';
NOT : '!' | '~';

// Temporal operators
HOLD   : 'H';
//WITHIN : 'W';
CONCAT : '*';

//match an integer
INT : ('0' | (('1'..'9')DIGIT*));

//match boolean constants
TRUE : 'True' | 'true';
FALSE : 'False' | 'false';

//match a proposition
PROP  : ((LWLETTER | HGLETTERALL)('_' | LETTER | DIGIT)*);

//match a line comment
LINECMT : ('//')(~('\n'|'\r'))* -> skip; //line comment

//match a whitespace
WS  : (('\n' | '\r' | '\f' | '\t' | /*'\v' |*/ ' ')+) -> skip; //whitespaces
