# $ANTLR 3.5.2 bound.g 2023-08-24 12:33:17

import sys
from antlr3 import *
from antlr3.tree import *

from antlr3.compat import set, frozenset

        
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



# for convenience in actions
HIDDEN = BaseRecognizer.HIDDEN

# token types
EOF=-1
T__21=21
T__22=22
T__23=23
T__24=24
T__25=25
T__26=26
AND=4
CONCAT=5
DIGIT=6
FALSE=7
HGLETTER=8
HGLETTERALL=9
HOLD=10
INT=11
LETTER=12
LINECMT=13
LWLETTER=14
NOT=15
OR=16
PROP=17
TRUE=18
WITHIN=19
WS=20

# token names
tokenNames = [
    "<invalid>", "<EOR>", "<DOWN>", "<UP>",
    "AND", "CONCAT", "DIGIT", "FALSE", "HGLETTER", "HGLETTERALL", "HOLD", 
    "INT", "LETTER", "LINECMT", "LWLETTER", "NOT", "OR", "PROP", "TRUE", 
    "WITHIN", "WS", "'('", "')'", "','", "'['", "']'", "'^'"
]




class bound(TreeParser):
    grammarFileName = "bound.g"
    api_version = 1
    tokenNames = tokenNames

    def __init__(self, input, state=None, *args, **kwargs):
        if state is None:
            state = RecognizerSharedState()

        super(bound, self).__init__(input, state, *args, **kwargs)




        self.delegates = []




             
    bound = 0

    def getBound(self):
        return self.bound



    # $ANTLR start "eval"
    # bound.g:58:1: eval : formula ;
    def eval(self, ):
        formula1 = None

        try:
            try:
                # bound.g:58:5: ( formula )
                # bound.g:58:9: formula
                pass 
                self._state.following.append(self.FOLLOW_formula_in_eval62)
                formula1 = self.formula()

                self._state.following.pop()

                #action start
                self.bound = formula1;
                #action end





            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)

        finally:
            pass
        return 

    # $ANTLR end "eval"



    # $ANTLR start "formula"
    # bound.g:61:1: formula returns [value] : ( ^( OR a= formula b= formula ) | ^( AND a= formula b= formula ) | ^( NOT a= formula ) | ^( CONCAT a= formula b= formula ) | ^( HOLD INT PROP ) | ^( HOLD INT ^( NOT PROP ) ) | ^( WITHIN phi= formula low= INT high= INT ) | PROP | TRUE | FALSE );
    def formula(self, ):
        value = None


        low = None
        high = None
        INT2 = None
        INT3 = None
        a = None
        b = None
        phi = None

        try:
            try:
                # bound.g:62:5: ( ^( OR a= formula b= formula ) | ^( AND a= formula b= formula ) | ^( NOT a= formula ) | ^( CONCAT a= formula b= formula ) | ^( HOLD INT PROP ) | ^( HOLD INT ^( NOT PROP ) ) | ^( WITHIN phi= formula low= INT high= INT ) | PROP | TRUE | FALSE )
                alt1 = 10
                LA1 = self.input.LA(1)
                if LA1 == OR:
                    alt1 = 1
                elif LA1 == AND:
                    alt1 = 2
                elif LA1 == NOT:
                    alt1 = 3
                elif LA1 == CONCAT:
                    alt1 = 4
                elif LA1 == HOLD:
                    LA1_5 = self.input.LA(2)

                    if (LA1_5 == 2) :
                        LA1_10 = self.input.LA(3)

                        if (LA1_10 == INT) :
                            LA1_11 = self.input.LA(4)

                            if (LA1_11 == PROP) :
                                alt1 = 5
                            elif (LA1_11 == NOT) :
                                alt1 = 6
                            else:
                                nvae = NoViableAltException("", 1, 11, self.input)

                                raise nvae


                        else:
                            nvae = NoViableAltException("", 1, 10, self.input)

                            raise nvae


                    else:
                        nvae = NoViableAltException("", 1, 5, self.input)

                        raise nvae


                elif LA1 == WITHIN:
                    alt1 = 7
                elif LA1 == PROP:
                    alt1 = 8
                elif LA1 == TRUE:
                    alt1 = 9
                elif LA1 == FALSE:
                    alt1 = 10
                else:
                    nvae = NoViableAltException("", 1, 0, self.input)

                    raise nvae


                if alt1 == 1:
                    # bound.g:62:9: ^( OR a= formula b= formula )
                    pass 
                    self.match(self.input, OR, self.FOLLOW_OR_in_formula88)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula93)
                    a = self.formula()

                    self._state.following.pop()

                    self._state.following.append(self.FOLLOW_formula_in_formula97)
                    b = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    value = (min(a[0], b[0]), max(a[1], b[1]))
                    #action end



                elif alt1 == 2:
                    # bound.g:63:9: ^( AND a= formula b= formula )
                    pass 
                    self.match(self.input, AND, self.FOLLOW_AND_in_formula111)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula115)
                    a = self.formula()

                    self._state.following.pop()

                    self._state.following.append(self.FOLLOW_formula_in_formula119)
                    b = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    value = (max(a[0], b[0]), max(a[1], b[1]))
                    #action end



                elif alt1 == 3:
                    # bound.g:64:9: ^( NOT a= formula )
                    pass 
                    self.match(self.input, NOT, self.FOLLOW_NOT_in_formula133)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula137)
                    a = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    value = a
                    #action end



                elif alt1 == 4:
                    # bound.g:65:9: ^( CONCAT a= formula b= formula )
                    pass 
                    self.match(self.input, CONCAT, self.FOLLOW_CONCAT_in_formula161)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula165)
                    a = self.formula()

                    self._state.following.pop()

                    self._state.following.append(self.FOLLOW_formula_in_formula169)
                    b = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    value = [x+y+1 for x, y in zip(a, b)]
                    #action end



                elif alt1 == 5:
                    # bound.g:66:9: ^( HOLD INT PROP )
                    pass 
                    self.match(self.input, HOLD, self.FOLLOW_HOLD_in_formula183)

                    self.match(self.input, DOWN, None)
                    INT2 = self.match(self.input, INT, self.FOLLOW_INT_in_formula185)

                    self.match(self.input, PROP, self.FOLLOW_PROP_in_formula187)

                    self.match(self.input, UP, None)


                    #action start
                    value = (int(INT2.text), int(INT2.text))
                    #action end



                elif alt1 == 6:
                    # bound.g:67:9: ^( HOLD INT ^( NOT PROP ) )
                    pass 
                    self.match(self.input, HOLD, self.FOLLOW_HOLD_in_formula208)

                    self.match(self.input, DOWN, None)
                    INT3 = self.match(self.input, INT, self.FOLLOW_INT_in_formula210)

                    self.match(self.input, NOT, self.FOLLOW_NOT_in_formula213)

                    self.match(self.input, DOWN, None)
                    self.match(self.input, PROP, self.FOLLOW_PROP_in_formula215)

                    self.match(self.input, UP, None)


                    self.match(self.input, UP, None)


                    #action start
                    value = (int(INT3.text), int(INT3.text))
                    #action end



                elif alt1 == 7:
                    # bound.g:68:9: ^( WITHIN phi= formula low= INT high= INT )
                    pass 
                    self.match(self.input, WITHIN, self.FOLLOW_WITHIN_in_formula230)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula234)
                    phi = self.formula()

                    self._state.following.pop()

                    low = self.match(self.input, INT, self.FOLLOW_INT_in_formula238)

                    high = self.match(self.input, INT, self.FOLLOW_INT_in_formula242)

                    self.match(self.input, UP, None)


                    #action start
                                                                   
                    value = (int(low.text)+phi[0], int(high.text)) # - int(low.text)
                    if phi[1] > int(high.text) - int(low.text):
                        raise ValueError("Within operator deadline is invalid!")
                            
                    #action end



                elif alt1 == 8:
                    # bound.g:73:9: PROP
                    pass 
                    self.match(self.input, PROP, self.FOLLOW_PROP_in_formula255)

                    #action start
                    value =  (0, 0)
                    #action end



                elif alt1 == 9:
                    # bound.g:74:9: TRUE
                    pass 
                    self.match(self.input, TRUE, self.FOLLOW_TRUE_in_formula290)

                    #action start
                    value =  (0, 0)
                    #action end



                elif alt1 == 10:
                    # bound.g:75:9: FALSE
                    pass 
                    self.match(self.input, FALSE, self.FOLLOW_FALSE_in_formula325)

                    #action start
                    value =  (0, 0)
                    #action end




            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)

        finally:
            pass
        return value

    # $ANTLR end "formula"



 

    FOLLOW_formula_in_eval62 = frozenset([1])
    FOLLOW_OR_in_formula88 = frozenset([2])
    FOLLOW_formula_in_formula93 = frozenset([4, 5, 7, 10, 15, 16, 17, 18, 19])
    FOLLOW_formula_in_formula97 = frozenset([3])
    FOLLOW_AND_in_formula111 = frozenset([2])
    FOLLOW_formula_in_formula115 = frozenset([4, 5, 7, 10, 15, 16, 17, 18, 19])
    FOLLOW_formula_in_formula119 = frozenset([3])
    FOLLOW_NOT_in_formula133 = frozenset([2])
    FOLLOW_formula_in_formula137 = frozenset([3])
    FOLLOW_CONCAT_in_formula161 = frozenset([2])
    FOLLOW_formula_in_formula165 = frozenset([4, 5, 7, 10, 15, 16, 17, 18, 19])
    FOLLOW_formula_in_formula169 = frozenset([3])
    FOLLOW_HOLD_in_formula183 = frozenset([2])
    FOLLOW_INT_in_formula185 = frozenset([17])
    FOLLOW_PROP_in_formula187 = frozenset([3])
    FOLLOW_HOLD_in_formula208 = frozenset([2])
    FOLLOW_INT_in_formula210 = frozenset([15])
    FOLLOW_NOT_in_formula213 = frozenset([2])
    FOLLOW_PROP_in_formula215 = frozenset([3])
    FOLLOW_WITHIN_in_formula230 = frozenset([2])
    FOLLOW_formula_in_formula234 = frozenset([11])
    FOLLOW_INT_in_formula238 = frozenset([11])
    FOLLOW_INT_in_formula242 = frozenset([3])
    FOLLOW_PROP_in_formula255 = frozenset([1])
    FOLLOW_TRUE_in_formula290 = frozenset([1])
    FOLLOW_FALSE_in_formula325 = frozenset([1])



def main(argv, stdin=sys.stdin, stdout=sys.stdout, stderr=sys.stderr):
    from antlr3.main import WalkerMain
    main = WalkerMain(bound)

    main.stdin = stdin
    main.stdout = stdout
    main.stderr = stderr
    main.execute(argv)



if __name__ == '__main__':
    main(sys.argv)
