# $ANTLR 3.5.2 twtl2dfa.g 2023-08-24 12:34:36

import sys
from antlr3 import *
from antlr3.tree import *

from antlr3.compat import set, frozenset

        
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




class twtl2dfa(TreeParser):
    grammarFileName = "twtl2dfa.g"
    api_version = 1
    tokenNames = tokenNames

    def __init__(self, input, state=None, *args, **kwargs):
        if state is None:
            state = RecognizerSharedState()

        super(twtl2dfa, self).__init__(input, state, *args, **kwargs)




        self.delegates = []




             
    dfa=None
    props = None

    def getDFA(self):
        return self.dfa



    # $ANTLR start "eval"
    # twtl2dfa.g:62:1: eval : formula ;
    def eval(self, ):
        formula1 = None

        try:
            try:
                # twtl2dfa.g:62:5: ( formula )
                # twtl2dfa.g:62:9: formula
                pass 
                self._state.following.append(self.FOLLOW_formula_in_eval62)
                formula1 = self.formula()

                self._state.following.pop()

                #action start
                self.dfa = formula1;
                #action end





            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)

        finally:
            pass
        return 

    # $ANTLR end "eval"



    # $ANTLR start "formula"
    # twtl2dfa.g:65:1: formula returns [dfa] : ( ^( OR a= formula b= formula ) | ^( AND a= formula b= formula ) | ^( NOT a= formula ) | ^( CONCAT a= formula b= formula ) | ^( HOLD INT p= PROP ) | ^( HOLD INT ^( NOT p= PROP ) ) | ^( WITHIN phi= formula low= INT high= INT ) | PROP | TRUE | FALSE );
    def formula(self, ):
        dfa = None


        p = None
        low = None
        high = None
        INT2 = None
        INT3 = None
        PROP4 = None
        a = None
        b = None
        phi = None

        try:
            try:
                # twtl2dfa.g:66:5: ( ^( OR a= formula b= formula ) | ^( AND a= formula b= formula ) | ^( NOT a= formula ) | ^( CONCAT a= formula b= formula ) | ^( HOLD INT p= PROP ) | ^( HOLD INT ^( NOT p= PROP ) ) | ^( WITHIN phi= formula low= INT high= INT ) | PROP | TRUE | FALSE )
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
                    # twtl2dfa.g:66:9: ^( OR a= formula b= formula )
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
                    dfa = union(a, b)
                    #action end



                elif alt1 == 2:
                    # twtl2dfa.g:67:9: ^( AND a= formula b= formula )
                    pass 
                    self.match(self.input, AND, self.FOLLOW_AND_in_formula112)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula116)
                    a = self.formula()

                    self._state.following.pop()

                    self._state.following.append(self.FOLLOW_formula_in_formula120)
                    b = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    dfa = intersection(a, b)
                    #action end



                elif alt1 == 3:
                    # twtl2dfa.g:68:9: ^( NOT a= formula )
                    pass 
                    self.match(self.input, NOT, self.FOLLOW_NOT_in_formula135)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula139)
                    a = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    dfa = complement(a)
                    #action end



                elif alt1 == 4:
                    # twtl2dfa.g:69:9: ^( CONCAT a= formula b= formula )
                    pass 
                    self.match(self.input, CONCAT, self.FOLLOW_CONCAT_in_formula164)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula168)
                    a = self.formula()

                    self._state.following.pop()

                    self._state.following.append(self.FOLLOW_formula_in_formula172)
                    b = self.formula()

                    self._state.following.pop()

                    self.match(self.input, UP, None)


                    #action start
                    dfa = concatenation(a, b)
                    #action end



                elif alt1 == 5:
                    # twtl2dfa.g:70:9: ^( HOLD INT p= PROP )
                    pass 
                    self.match(self.input, HOLD, self.FOLLOW_HOLD_in_formula187)

                    self.match(self.input, DOWN, None)
                    INT2 = self.match(self.input, INT, self.FOLLOW_INT_in_formula189)

                    p = self.match(self.input, PROP, self.FOLLOW_PROP_in_formula193)

                    self.match(self.input, UP, None)


                    #action start
                                               
                    dfa = hold(self.props, p.text, int(INT2.text), negation=False)
                            
                    #action end



                elif alt1 == 6:
                    # twtl2dfa.g:73:9: ^( HOLD INT ^( NOT p= PROP ) )
                    pass 
                    self.match(self.input, HOLD, self.FOLLOW_HOLD_in_formula207)

                    self.match(self.input, DOWN, None)
                    INT3 = self.match(self.input, INT, self.FOLLOW_INT_in_formula209)

                    self.match(self.input, NOT, self.FOLLOW_NOT_in_formula212)

                    self.match(self.input, DOWN, None)
                    p = self.match(self.input, PROP, self.FOLLOW_PROP_in_formula216)

                    self.match(self.input, UP, None)


                    self.match(self.input, UP, None)


                    #action start
                                                      
                    dfa = hold(self.props, p.text, int(INT3.text), negation=True)
                            
                    #action end



                elif alt1 == 7:
                    # twtl2dfa.g:76:9: ^( WITHIN phi= formula low= INT high= INT )
                    pass 
                    self.match(self.input, WITHIN, self.FOLLOW_WITHIN_in_formula231)

                    self.match(self.input, DOWN, None)
                    self._state.following.append(self.FOLLOW_formula_in_formula235)
                    phi = self.formula()

                    self._state.following.pop()

                    low = self.match(self.input, INT, self.FOLLOW_INT_in_formula239)

                    high = self.match(self.input, INT, self.FOLLOW_INT_in_formula243)

                    self.match(self.input, UP, None)


                    #action start
                                                                   
                    dfa = within(phi, int(low.text), int(high.text))
                            
                    #action end



                elif alt1 == 8:
                    # twtl2dfa.g:79:9: PROP
                    pass 
                    PROP4 = self.match(self.input, PROP, self.FOLLOW_PROP_in_formula256)

                    #action start
                    dfa = accept_prop(self.props, prop=PROP4)
                    #action end



                elif alt1 == 9:
                    # twtl2dfa.g:80:9: TRUE
                    pass 
                    self.match(self.input, TRUE, self.FOLLOW_TRUE_in_formula291)

                    #action start
                    dfa = accept_prop(self.props, boolean=True)
                    #action end



                elif alt1 == 10:
                    # twtl2dfa.g:81:9: FALSE
                    pass 
                    self.match(self.input, FALSE, self.FOLLOW_FALSE_in_formula326)

                    #action start
                    dfa = accept_prop(self.props, boolean=False)
                    #action end




            except RecognitionException, re:
                self.reportError(re)
                self.recover(self.input, re)

        finally:
            pass
        return dfa

    # $ANTLR end "formula"



 

    FOLLOW_formula_in_eval62 = frozenset([1])
    FOLLOW_OR_in_formula88 = frozenset([2])
    FOLLOW_formula_in_formula93 = frozenset([4, 5, 7, 10, 15, 16, 17, 18, 19])
    FOLLOW_formula_in_formula97 = frozenset([3])
    FOLLOW_AND_in_formula112 = frozenset([2])
    FOLLOW_formula_in_formula116 = frozenset([4, 5, 7, 10, 15, 16, 17, 18, 19])
    FOLLOW_formula_in_formula120 = frozenset([3])
    FOLLOW_NOT_in_formula135 = frozenset([2])
    FOLLOW_formula_in_formula139 = frozenset([3])
    FOLLOW_CONCAT_in_formula164 = frozenset([2])
    FOLLOW_formula_in_formula168 = frozenset([4, 5, 7, 10, 15, 16, 17, 18, 19])
    FOLLOW_formula_in_formula172 = frozenset([3])
    FOLLOW_HOLD_in_formula187 = frozenset([2])
    FOLLOW_INT_in_formula189 = frozenset([17])
    FOLLOW_PROP_in_formula193 = frozenset([3])
    FOLLOW_HOLD_in_formula207 = frozenset([2])
    FOLLOW_INT_in_formula209 = frozenset([15])
    FOLLOW_NOT_in_formula212 = frozenset([2])
    FOLLOW_PROP_in_formula216 = frozenset([3])
    FOLLOW_WITHIN_in_formula231 = frozenset([2])
    FOLLOW_formula_in_formula235 = frozenset([11])
    FOLLOW_INT_in_formula239 = frozenset([11])
    FOLLOW_INT_in_formula243 = frozenset([3])
    FOLLOW_PROP_in_formula256 = frozenset([1])
    FOLLOW_TRUE_in_formula291 = frozenset([1])
    FOLLOW_FALSE_in_formula326 = frozenset([1])



def main(argv, stdin=sys.stdin, stdout=sys.stdout, stderr=sys.stderr):
    from antlr3.main import WalkerMain
    main = WalkerMain(twtl2dfa)

    main.stdin = stdin
    main.stdout = stdout
    main.stderr = stderr
    main.execute(argv)



if __name__ == '__main__':
    main(sys.argv)
