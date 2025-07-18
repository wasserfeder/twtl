

def translate(formula, optimize=True):

    # Generate the lexer and parser for the given TWTL formula
    lexer = twtlLexer(ANTLRStringStream(formula))
    lexer.setAlphabet(set())
    tokens = CommonTokenStream(lexer)
    parser = twtlParser(tokens)
    phi = parser.formula()

    # CommonTree
    t = phi.tree
    alphabet = lexer.getAlphabet()
    result= [alphabet]

    if DFAType.Infinity in kind:
        setDFAType(DFAType.Infinity)
        setOptimizationFlag(optimize)
        nodes = CommonTreeNodeStream(t)
        nodes.setTokenStream(tokens)
        translator = twtl2dfa(nodes)
        translator.props = alphabet
        translator.eval()
        dfa_inf = translator.getDFA()
        dfa_inf.kind = DFAType.Infinity
        result.append(dfa_inf)

    if norm: # compute TWTL bound
        nodes = CommonTreeNodeStream(t)
        nodes.setTokenStream(tokens)
        boundEvaluator = bound(nodes)
        boundEvaluator.eval()
        result.append(boundEvaluator.getBound())

    pdfa = dfa_inf

    return tuple(result)


def temporal_relaxation(word, formula=None, dfa=None):

    if dfa is None:
        _, dfa = translate(formula, kind=DFAType.Infinity, optimize=False)
    assert dfa.kind == DFAType.Infinity

    # initialize tree
    _init_tree(dfa.tree)
    prev_state = None
    state = dfa.init.keys()[0]
    prev_w = set()
    for w in word + [set([])]: # hack to catch the last state
        # start/stop counters and increment all active counters
        _update_tree(dfa.tree, state, prev_state, dfa.bitmap_of_props(prev_w))
        if state in dfa.final:
            break
        else: # compute next state
            r = dfa.next_states(state, w)
            assert len(r) == 1, 'Should be deterministic!'
            prev_state = state
            state = r[0]
        prev_w = w
    # substract deadlines from counter values to obtain the tau values
    return _eval_relaxation(dfa.tree)
