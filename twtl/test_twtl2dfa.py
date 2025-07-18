  
import twtl
import dfa

formula = '[H^0 A]^ [0,2]'
_, dfa_1, dfa_inf, bdd = twtl.translate(formula, kind='both', norm=True)


formula = '[H^0 A ]^ [0,2] | [H^0 B]^[0,2]'
_, dfa_2, dfa_inf, bdd = twtl.translate(formula, kind='both', norm=True)



product = dfa.union(dfa_1, dfa_2)

print(product)