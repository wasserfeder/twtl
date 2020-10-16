/**
 *  Grammar file for Time Window Temporal Logic.
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
grammar twtl;

options {
    language = Python;
    output = AST;
    ASTLabelType = CommonTree;
}

tokens {
    // Boolean operators
    AND = '&';
    OR  = '|';
    NOT = '!';

    // Temporal operators
    HOLD   = 'H';
    WITHIN = 'W';
    CONCAT = '*';
}

@header {
license_text='''
    Parser for TWTL formulae.
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
}

@lexer::header {
license_text='''
    Lexer for TWTL formulae.
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

@lexer::members {
    def getAlphabet(self):
        return self.alphabet

    def setAlphabet(self, alphabet):
        self.alphabet = alphabet
}

//parser entry point
prog:   formula ;

formula :   disjunction;

disjunction //parse a disjunction operation or skip
    : conjunction (OR^ conjunction)*
    ;

conjunction //parse a conjunction operation or skip
    :   concatenation (AND^ concatenation)*
    ;

concatenation //parse a concatenation operation or skip
    :   temporal (CONCAT^ temporal)*
    ;

temporal //parse hold or within operators or skip
    : HOLD^ '^'! duration=INT PROP
    | HOLD '^' duration=INT NOT p=PROP -> ^(HOLD $duration ^(NOT $p))
    | ('[' phi=formula ']' '^' '[' low=INT ',' high=INT ']') -> ^(WITHIN['W'] $phi $low $high)
    | negation
    ;

negation //parse a not operation
    : (NOT^)? atom
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

//tokens
//match an integer
INT : ('0' | (('1'..'9')DIGIT*))
    ;

//match boolean constants
TRUE : 'True' | 'true';
FALSE : 'False' | 'false';

//match a proposition
PROP  : ((LWLETTER | HGLETTERALL)('_' | LETTER | DIGIT)*)
    {
        if str($text).lower() not in ('true', 'false'):
            self.alphabet.add(str($text));
    }
    ;

//match a line comment
LINECMT : ('//')(~('\n'|'\r'))*
    { self.skip()}
    ; //line comment

//match a whitespace
WS  : (('\n' | '\r' | '\f' | '\t' | /*'\v' |*/ ' ')+)
    { self.skip()}
    ; //whitespaces
