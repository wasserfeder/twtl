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
    Parser for TWTL formulae.
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

@lexer::header {
license_text='''
    Lexer for TWTL formulae.
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
prog:   formula ;

formula :   disjunction;

disjunction //parse a disjunction operation or skip
    : conjunction (OR conjunction)*
    ;

conjunction //parse a conjunction operation or skip
    :   concatenation (AND concatenation)*
    ;

concatenation //parse a concatenation operation or skip
    :   temporal (CONCAT temporal)*
    ;

temporal //parse hold or within operators or skip
    : HOLD '^'! duration=INT PROP
    | HOLD '^' duration=INT NOT p=PROP
    | ('[' phi=formula ']' '^' '[' low=INT ',' high=INT ']')
    | negation
    ;

negation //parse a not operation
    : (NOT)? atom
    ;

atom //parse proposition, constants or formula
    : TRUE //parse true bool constant
    | FALSE //parse false bool constant
    | PROP //parse proposition
    | '('! formula ')'! //parse parenthesis
    ;

//fragments
fragment
DIGIT       : ('0'..'9'); //digit
fragment
LWLETTER    : ('a'..'z'); //lower case letter
fragment
HGLETTER    : ('A'..'Z'); //higher case letter
fragment
HGLETTERALL : ('A'..'G') | ('I' .. 'V') | ('X' .. 'Z'); //allowed higher case letter
fragment
LETTER      : LWLETTER | HGLETTER; //letter

// Boolean operators
AND = '&';
OR  = '|';
NOT = '!';

// Temporal operators
HOLD   = 'H';
//WITHIN = 'W';
CONCAT = '*';

//tokens
//match an integer
INT : ('0' | (('1'..'9')DIGIT*))
    ;

//match boolean constants
TRUE : 'True' | 'true';
FALSE : 'False' | 'false';

//match a proposition
PROP  : ((LWLETTER | HGLETTERALL)('_' | LETTER | DIGIT)*);

//match a line comment
LINECMT : ('//')(~('\n'|'\r'))* -> skip; //line comment

//match a whitespace
WS  : (('\n' | '\r' | '\f' | '\t' | /*'\v' |*/ ' ')+) -> skip; //whitespaces
