# Generated from mtl.g4 by ANTLR 4.7.1
from antlr4 import *

'''
 Copyright (C) 2015-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 Explainable Robotics Lab, Lehigh University
 See license.txt file for license information.
'''


# This class defines a complete generic visitor for a parse tree produced by mtlParser.

class mtlVisitor(ParseTreeVisitor):

    # Visit a parse tree produced by mtlParser#booleanPred.
    def visitBooleanPred(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by mtlParser#formula.
    def visitFormula(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by mtlParser#parprop.
    def visitParprop(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by mtlParser#booleanExpr.
    def visitBooleanExpr(self, ctx):
        return self.visitChildren(ctx)


